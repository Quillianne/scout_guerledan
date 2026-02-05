from django.contrib import admin
from django.urls import path

from replay import views as replay_views

urlpatterns = [
    path("admin/", admin.site.urls),
    path("", replay_views.index, name="index"),
    path("api/logs/", replay_views.api_logs, name="api_logs"),
    path("api/replay/", replay_views.api_replay, name="api_replay"),
]
