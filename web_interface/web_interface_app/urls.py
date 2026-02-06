from django.contrib import admin
from django.urls import path

from replay import views as replay_views

urlpatterns = [
    path("admin/", admin.site.urls),
    path("", replay_views.index, name="home"),
    path("logs/", replay_views.logs_view, name="logs_view"),
    path("manager/", replay_views.manager_view, name="manager_view"),
    path("interval/", replay_views.interval_view, name="interval_view"),
    path("api/logs/", replay_views.api_logs, name="api_logs"),
    path("api/logs/delete/", replay_views.api_logs_delete, name="api_logs_delete"),
    path("api/replay/", replay_views.api_replay, name="api_replay"),
    path("api/boats/", replay_views.api_boats, name="api_boats"),
    path("api/boats/save/", replay_views.api_boats_save, name="api_boats_save"),
    path("api/boats/gps/", replay_views.api_boats_gps, name="api_boats_gps"),
    path("api/boats/action/", replay_views.api_boat_action, name="api_boat_action"),
    path("api/interval/reset/", replay_views.api_interval_reset, name="api_interval_reset"),
    path("api/interval/step/", replay_views.api_interval_step, name="api_interval_step"),
    path("api/interval/log/start/", replay_views.api_interval_log_start, name="api_interval_log_start"),
    path("api/interval/log/stop/", replay_views.api_interval_log_stop, name="api_interval_log_stop"),
]
