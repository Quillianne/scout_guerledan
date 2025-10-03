// blueboat_rtt.cpp
// BlueBoat Micron Data Modem — Application-level RTT (addressed) distance demo (C++)
// ==================================================================================
// ⚠️ Accuracy caveat: This "RTT applicatif" provides only coarse distance estimates.
// For sub-meter ranging, use the Micron Modem built-in `rng` or a USBL (MicronNav).
//
// Build:
//   g++ -std=c++17 -O2 -pthread blueboat_rtt.cpp -o blueboat_rtt
//
// Run (one per boat; adjust port):
//   ./blueboat_rtt --id 1 --port /dev/ttyUSB0 --c 1500 --offset-ms 0.0
//   ./blueboat_rtt --id 2 --port /dev/ttyUSB0 --c 1500 --offset-ms 0.0
//   ./blueboat_rtt --id 3 --port /dev/ttyUSB0 --c 1500 --offset-ms 0.0
//
// Protocol frame (compact):
//   MAGIC (0xA5) | LEN (1B) | TYPE (1B) | PAYLOAD (LEN B) | CRC8 (1B)
// CRC8 over [LEN | TYPE | PAYLOAD]
//
// Types/Payloads:
//   PING (0x01): [SEQ(1), SRC(1), DST(1)]
//   PONG (0x02): [SEQ(1), SRC(1), DST(1), PROC_US(2, big-endian)]
//   DIST (0x03): [PAIR(1: 0x12/0x13/0x23), DIST_CM(2, big-endian)]
//   CMD  (0x04): [CODE(1), ARG(1 optional)]
//
// TDMA (default 12 s):
//   ID=1 (master): measure d12 & d13; broadcast both; ask ID=2 to measure d23 and broadcast.
//
// Serial setup uses POSIX termios; default 9600 8N1.
//
// License: MIT
#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cinttypes>
#include <csignal>
#include <cstring>
#include <deque>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

// ---------- Utilities ----------
static inline uint64_t now_ns() {
    using namespace std::chrono;
    return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}
static inline double now_s() {
    return now_ns() / 1e9;
}

// ---------- CRC8 (poly 0x07) ----------
uint8_t crc8(const std::vector<uint8_t>& data, uint8_t poly=0x07, uint8_t init=0x00) {
    uint8_t crc = init;
    for (uint8_t b : data) {
        crc ^= b;
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ poly);
            else crc <<= 1;
        }
    }
    return crc;
}

// ---------- Serial Port (POSIX) ----------
class SerialPort {
public:
    SerialPort(const std::string& path, int baud=9600) {
        fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            throw std::runtime_error("Failed to open serial port: " + path + " : " + std::strerror(errno));
        }
        // Configure termios
        termios tty{};
        if (tcgetattr(fd_, &tty) != 0) {
            throw std::runtime_error("tcgetattr failed");
        }
        cfmakeraw(&tty);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_oflag = 0;
        tty.c_lflag = 0;
        tty.c_iflag = 0;
        tty.c_cc[VTIME] = 0; // read timeout deciseconds (0 = non-blocking)
        tty.c_cc[VMIN]  = 0; // non-blocking

        speed_t speed = B9600;
        switch (baud) {
            case 4800: speed = B4800; break;
            case 9600: speed = B9600; break;
            case 19200: speed = B19200; break;
            case 38400: speed = B38400; break;
            case 57600: speed = B57600; break;
            case 115200: speed = B115200; break;
            default: speed = B9600; break; // typical for Micron serial
        }
        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            throw std::runtime_error("tcsetattr failed");
        }
    }
    ~SerialPort() {
        if (fd_ >= 0) ::close(fd_);
    }

    ssize_t write_bytes(const uint8_t* data, size_t n) {
        if (fd_ < 0) return -1;
        ssize_t w = ::write(fd_, data, n);
        return w;
    }
    ssize_t read_bytes(uint8_t* data, size_t n) {
        if (fd_ < 0) return -1;
        return ::read(fd_, data, n);
    }
    int fd() const { return fd_; }

private:
    int fd_{-1};
};

// ---------- Framing ----------
constexpr uint8_t MAGIC = 0xA5;
enum MsgType : uint8_t { T_PING=0x01, T_PONG=0x02, T_DIST=0x03, T_CMD=0x04 };
constexpr uint8_t CMD_MEAS_23 = 0x31;

struct Frame {
    uint8_t type;
    std::vector<uint8_t> payload;
};

std::vector<uint8_t> build_frame(uint8_t type, const std::vector<uint8_t>& payload) {
    if (payload.size() > 255) throw std::runtime_error("payload too large");
    std::vector<uint8_t> out;
    out.reserve(1 + 1 + 1 + payload.size() + 1);
    out.push_back(MAGIC);
    uint8_t len = (uint8_t)payload.size();
    out.push_back(len);
    out.push_back(type);
    // CRC over LEN|TYPE|PAYLOAD
    std::vector<uint8_t> crc_data;
    crc_data.reserve(2 + payload.size());
    crc_data.push_back(len);
    crc_data.push_back(type);
    for (auto b : payload) out.push_back(b), crc_data.push_back(b);
    out.push_back(crc8(crc_data));
    return out;
}

// Parser stateful over a byte ring
class FrameParser {
public:
    void feed(const uint8_t* data, size_t n) {
        buf_.insert(buf_.end(), data, data + n);
    }
    // Extract valid frames
    std::vector<Frame> parse_available() {
        std::vector<Frame> out;
        size_t i = 0;
        while (true) {
            // seek MAGIC
            while (i < buf_.size() && buf_[i] != MAGIC) ++i;
            if (i >= buf_.size()) {
                // drop consumed
                buf_.erase(buf_.begin(), buf_.begin() + i);
                break;
            }
            if (i + 3 >= buf_.size()) {
                // need at least MAGIC LEN TYPE CRC
                break;
            }
            size_t magic_i = i;
            uint8_t len = buf_[i+1];
            size_t full = 1 + 1 + 1 + len + 1;
            if (magic_i + full > buf_.size()) {
                break; // incomplete
            }
            // slice
            uint8_t type = buf_[i+2];
            const uint8_t* payload = &buf_[i+3];
            uint8_t crc = buf_[i+3+len];
            // verify
            std::vector<uint8_t> crc_data;
            crc_data.reserve(2 + len);
            crc_data.push_back(len);
            crc_data.push_back(type);
            for (size_t k=0;k<len;++k) crc_data.push_back(payload[k]);
            if (crc8(crc_data) == crc) {
                Frame f;
                f.type = type;
                f.payload.assign(payload, payload + len);
                out.push_back(std::move(f));
                // consume
                buf_.erase(buf_.begin(), buf_.begin() + magic_i + full);
                i = 0;
            } else {
                // bad frame; discard this MAGIC and continue
                i = magic_i + 1;
            }
        }
        return out;
    }
private:
    std::vector<uint8_t> buf_;
};

// ---------- Thread-safe inbox ----------
struct InboxItem {
    Frame frame;
    double t_arrival;
};

class Inbox {
public:
    void push(const Frame& f, double t_arrival) {
        std::lock_guard<std::mutex> lk(m_);
        q_.push_back({f, t_arrival});
    }
    // Pop first matching predicate within timeout (s)
    std::optional<InboxItem> pop_match(double timeout_s,
        std::function<bool(const InboxItem&)> pred)
    {
        auto t0 = now_s();
        while (now_s() - t0 < timeout_s) {
            {
                std::lock_guard<std::mutex> lk(m_);
                for (size_t i=0;i<q_.size();++i) {
                    if (pred(q_[i])) {
                        InboxItem it = q_[i];
                        q_.erase(q_.begin() + i);
                        return it;
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return std::nullopt;
    }
private:
    std::mutex m_;
    std::deque<InboxItem> q_;
};

// ---------- SerialLink with reader thread ----------
class SerialLink {
public:
    SerialLink(const std::string& port, int baud)
    : sp_(port, baud), alive_(true) {
        reader_ = std::thread([this](){ this->reader_loop(); });
    }
    ~SerialLink() {
        alive_ = false;
        if (reader_.joinable()) reader_.join();
    }
    void send(uint8_t type, const std::vector<uint8_t>& payload) {
        auto frame = build_frame(type, payload);
        ssize_t w = sp_.write_bytes(frame.data(), frame.size());
        (void)w;
    }
    std::optional<InboxItem> recv_match(double timeout_s,
        std::function<bool(const InboxItem&)> pred)
    {
        return inbox_.pop_match(timeout_s, pred);
    }

private:
    void reader_loop() {
        FrameParser parser;
        std::vector<uint8_t> tmp(512);
        while (alive_) {
            ssize_t n = sp_.read_bytes(tmp.data(), tmp.size());
            if (n > 0) {
                parser.feed(tmp.data(), (size_t)n);
                auto frames = parser.parse_available();
                for (auto& f : frames) {
                    inbox_.push(f, now_s());
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
    }
    SerialPort sp_;
    std::atomic<bool> alive_;
    std::thread reader_;
    Inbox inbox_;
};

// ---------- Encoding helpers ----------
static inline std::vector<uint8_t> enc_ping(uint8_t seq, uint8_t src, uint8_t dst) {
    return {seq, src, dst};
}
static inline std::vector<uint8_t> enc_pong(uint8_t seq, uint8_t src, uint8_t dst, uint16_t proc_us) {
    // big-endian uint16
    return {seq, src, dst, (uint8_t)(proc_us >> 8), (uint8_t)(proc_us & 0xFF)};
}
static inline std::vector<uint8_t> enc_dist(uint8_t pair_code, uint16_t dist_cm) {
    return {pair_code, (uint8_t)(dist_cm >> 8), (uint8_t)(dist_cm & 0xFF)};
}
static inline std::vector<uint8_t> enc_cmd(uint8_t code, std::optional<uint8_t> arg=std::nullopt) {
    if (arg.has_value()) return {code, arg.value()};
    return {code};
}

struct DecPing { uint8_t seq, src, dst; };
struct DecPong { uint8_t seq, src, dst; uint16_t proc_us; };
struct DecDist { uint8_t pair; uint16_t dist_cm; };
struct DecCmd  { uint8_t code; std::optional<uint8_t> arg; };

static inline std::optional<DecPing> dec_ping(const std::vector<uint8_t>& p) {
    if (p.size()!=3) return std::nullopt;
    return DecPing{p[0],p[1],p[2]};
}
static inline std::optional<DecPong> dec_pong(const std::vector<uint8_t>& p) {
    if (p.size()!=5) return std::nullopt;
    uint16_t proc = (uint16_t(p[3])<<8) | p[4];
    return DecPong{p[0],p[1],p[2],proc};
}
static inline std::optional<DecDist> dec_dist(const std::vector<uint8_t>& p) {
    if (p.size()!=3) return std::nullopt;
    uint16_t cm = (uint16_t(p[1])<<8) | p[2];
    return DecDist{p[0], cm};
}
static inline std::optional<DecCmd> dec_cmd(const std::vector<uint8_t>& p) {
    if (p.size()==1) return DecCmd{p[0], std::nullopt};
    if (p.size()==2) return DecCmd{p[0], p[1]};
    return std::nullopt;
}

static inline uint8_t make_pair_code(uint8_t a, uint8_t b) {
    uint8_t lo = std::min(a,b), hi = std::max(a,b);
    return (uint8_t)((lo<<4) | hi); // 0x12, 0x13, 0x23
}

// ---------- Node ----------
class Node {
public:
    Node(uint8_t my_id, SerialLink& link, double c, double offset_s, int repeats, double timeout_s)
    : id_(my_id), link_(link), c_(c), offset_s_(offset_s), repeats_(repeats), timeout_s_(timeout_s)
    {
        assert(id_==1 || id_==2 || id_==3);
        listener_ = std::thread([this](){ this->listener_loop(); });
    }
    ~Node() {
        alive_ = false;
        if (listener_.joinable()) listener_.join();
    }

    double measure_to(uint8_t dst_id) {
        std::vector<double> samples;
        for (int i=0;i<repeats_;++i) {
            uint8_t seq = next_seq_();
            double t0 = now_s();
            link_.send(T_PING, enc_ping(seq, id_, dst_id));
            // wait matching PONG
            auto got = link_.recv_match(timeout_s_, [&](const InboxItem& it){
                if (it.frame.type != T_PONG) return false;
                auto d = dec_pong(it.frame.payload);
                if (!d) return false;
                return d->seq == seq && d->src == dst_id && d->dst == id_;
            });
            if (!got.has_value()) {
                std::cerr << "[ID" << int(id_) << "] PING seq=" << int(seq)
                          << " to ID" << int(dst_id) << ": timeout\n";
                continue;
            }
            double t1 = got->t_arrival;
            auto d = dec_pong(got->frame.payload).value();
            double rtt = t1 - t0;
            double rtt_adj = std::max(0.0, rtt - (d.proc_us / 1e6));
            samples.push_back(rtt_adj);
            std::cout << "[ID" << int(id_) << "] PING seq=" << int(seq)
                      << " to ID" << int(dst_id)
                      << ": RTT=" << rtt_adj*1e3 << " ms (raw " << rtt*1e3
                      << " ms, proc " << d.proc_us << " us)\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        if (samples.empty()) return -1.0;
        double rtt_min = *std::min_element(samples.begin(), samples.end());
        double d_m = std::max(0.0, 0.5 * c_ * std::max(0.0, rtt_min - offset_s_));
        std::cout << "[ID" << int(id_) << "] d(" << int(id_) << "->" << int(dst_id)
                  << ") ≈ " << d_m << " m (min-of-" << samples.size() << ", c=" << c_
                  << " m/s, offset=" << offset_s_*1e3 << " ms)\n";
        return d_m;
    }

    void broadcast_dist(uint8_t a, uint8_t b, double d_m) {
        uint8_t p = make_pair_code(a,b);
        uint16_t cm = (uint16_t)std::max(0, (int)std::llround(d_m * 100.0));
        link_.send(T_DIST, enc_dist(p, cm));
        std::cout << "[ID" << int(id_) << "] BROADCAST DIST d" << int(std::min(a,b))
                  << int(std::max(a,b)) << " = " << d_m << " m\n";
    }

    void send_cmd(uint8_t code, std::optional<uint8_t> arg=std::nullopt) {
        link_.send(T_CMD, enc_cmd(code, arg));
        std::cout << "[ID" << int(id_) << "] CMD sent: 0x" << std::hex << int(code) << std::dec << "\n";
    }

    void run_master(double cycle_s) {
        if (id_ != 1) throw std::runtime_error("run_master on non-master");
        while (alive_) {
            double t0 = now_s();
            double d12 = measure_to(2);
            if (d12 >= 0.0) broadcast_dist(1,2,d12);

            double d13 = measure_to(3);
            if (d13 >= 0.0) broadcast_dist(1,3,d13);

            send_cmd(CMD_MEAS_23); // ask ID2 to measure d23 and broadcast

            double elapsed = now_s() - t0;
            double remain = std::max(0.0, cycle_s - elapsed);
            std::this_thread::sleep_for(std::chrono::milliseconds((int)(remain*1000)));
        }
    }

    void run_slave(double cycle_s) {
        if (id_ == 1) throw std::runtime_error("run_slave on master");
        // Listener thread already handles PING and CMD.
        while (alive_) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

private:
    void listener_loop() {
        while (alive_) {
            auto got = link_.recv_match(0.2, [](const InboxItem&){ return true; });
            if (!got.has_value()) continue;
            auto& f = got->frame;
            if (f.type == T_PING) {
                auto p = dec_ping(f.payload);
                if (!p) continue;
                if (p->dst != id_) continue;
                // respond ASAP
                auto t_start = now_ns();
                uint16_t proc_us = (uint16_t)((now_ns() - t_start) / 1000);
                link_.send(T_PONG, enc_pong(p->seq, id_, p->src, proc_us));
                std::cout << "[ID" << int(id_) << "] -> PONG to ID" << int(p->src)
                          << ", seq=" << int(p->seq) << ", proc_us=" << proc_us << "\n";
            } else if (f.type == T_DIST) {
                auto d = dec_dist(f.payload);
                if (!d) continue;
                uint8_t a = d->pair >> 4, b = d->pair & 0x0F;
                double m = d->dist_cm / 100.0;
                std::cout << "[ID" << int(id_) << "] DIST update: d" << int(a) << int(b)
                          << " = " << m << " m (broadcast)\n";
            } else if (f.type == T_CMD) {
                auto c = dec_cmd(f.payload);
                if (!c) continue;
                handle_cmd(*c);
            }
        }
    }

    void handle_cmd(const DecCmd& c) {
        if (c.code == CMD_MEAS_23 && id_ == 2) {
            double d = measure_to(3);
            if (d >= 0.0) broadcast_dist(2,3,d);
        }
        // Extend with more commands if needed
    }

    uint8_t next_seq_() {
        return ++seq_;
    }

    uint8_t id_;
    SerialLink& link_;
    double c_;
    double offset_s_;
    int repeats_;
    double timeout_s_;
    std::atomic<bool> alive_{true};
    std::thread listener_;
    uint8_t seq_{0};
};

// ---------- Args ----------
struct Args {
    int id = -1;
    std::string port;
    int baud = 9600;
    double c = 1500.0;
    double offset_ms = 0.0;
    int repeats = 5;
    double cycle = 12.0;
    double timeout = 3.0;
};

void usage() {
    std::cout << "Usage: ./blueboat_rtt --id <1|2|3> --port /dev/ttyUSB0 [--baud 9600] "
                 "[--c 1500] [--offset-ms 0.0] [--repeats 5] [--cycle 12] [--timeout 3.0]\n";
}

Args parse_args(int argc, char** argv) {
    Args a;
    for (int i=1;i<argc;i++) {
        std::string s = argv[i];
        auto need = [&](int i){ if (i+1>=argc) throw std::runtime_error("Missing value after "+s); };
        if (s=="--id") { need(i); a.id = std::stoi(argv[++i]); }
        else if (s=="--port") { need(i); a.port = argv[++i]; }
        else if (s=="--baud") { need(i); a.baud = std::stoi(argv[++i]); }
        else if (s=="--c") { need(i); a.c = std::stod(argv[++i]); }
        else if (s=="--offset-ms") { need(i); a.offset_ms = std::stod(argv[++i]); }
        else if (s=="--repeats") { need(i); a.repeats = std::stoi(argv[++i]); }
        else if (s=="--cycle") { need(i); a.cycle = std::stod(argv[++i]); }
        else if (s=="--timeout") { need(i); a.timeout = std::stod(argv[++i]); }
        else if (s=="-h" || s=="--help") { usage(); std::exit(0); }
        else { std::cerr << "Unknown arg: " << s << "\n"; usage(); std::exit(1); }
    }
    if (a.id<1 || a.id>3 || a.port.empty()) { usage(); std::exit(1); }
    return a;
}

std::atomic<bool> g_stop{false};
void sigint_handler(int){ g_stop=true; }

int main(int argc, char** argv) {
    auto args = parse_args(argc, argv);
    std::signal(SIGINT, sigint_handler);

    try {
        SerialLink link(args.port, args.baud);
        Node node((uint8_t)args.id, link, args.c, args.offset_ms/1000.0, args.repeats, args.timeout);
        if (args.id == 1) node.run_master(args.cycle);
        else node.run_slave(args.cycle);
    } catch (const std::exception& e) {
        std::cerr << "Fatal: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
