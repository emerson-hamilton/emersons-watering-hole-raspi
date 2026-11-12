# Raspberry Pi deployment

These files reproduce the systemd services and sync script running on the camera Pi. Copy them to the Pi, fill in the placeholders, and install. Do not commit real hostnames, usernames, passwords, or keys to this repo.

## Files

| File | Installed to | Purpose |
|---|---|---|
| `emcam-rtsp.service` | `/etc/systemd/system/` | Runs `rtsp_server.py` (PIR capture, video motion check, MediaMTX stream) |
| `emcam-sync.service` | `/etc/systemd/system/` | Runs `sync_photos.sh` once |
| `emcam-sync.timer` | `/etc/systemd/system/` | Triggers `emcam-sync.service` every 60 seconds |
| `sync_photos.sh` | `/home/<pi-user>/Naturebytes/Scripts/` | rsyncs photos to the web server, then removes local copies that transferred |

## Install on the Pi

1. Copy the script into place and make it executable:
   ```bash
   cp deploy/sync_photos.sh /home/<pi-user>/Naturebytes/Scripts/
   chmod +x /home/<pi-user>/Naturebytes/Scripts/sync_photos.sh
   ```
2. Edit `REMOTE_USER` and `REMOTE_HOST` in the installed copy. Leave the repo copy with placeholders.
3. Copy the unit files and enable them:
   ```bash
   sudo cp deploy/emcam-rtsp.service deploy/emcam-sync.service deploy/emcam-sync.timer /etc/systemd/system/
   sudo systemctl daemon-reload
   sudo systemctl enable --now emcam-rtsp.service
   sudo systemctl enable --now emcam-sync.timer
   ```
4. Check that the services are running:
   ```bash
   systemctl status emcam-rtsp.service emcam-sync.timer
   systemctl list-timers emcam-sync.timer
   journalctl -u emcam-sync.service -n 20
   ```

## Server side (web server)

The SSH key used by the sync must be restricted by its `authorized_keys` entry so it can only write to the uploads folder:

```
command="/home/<web-user>/bin/rrsync /home/<web-user>/public_html/emersoncam/uploads",no-agent-forwarding,no-port-forwarding,no-pty,no-X11-forwarding,no-user-rc ssh-ed25519 <public key>
```

`rrsync` maps the rsync destination onto that folder, which is why `sync_photos.sh` does not name a remote path.

## Notes

- `RTSP_PASSWORD` is set in `rtsp_server.py`. Change it there and update the Synology's stream credentials to match. Do not commit a real password to this repo.
- MediaMTX is started by `rtsp_server.py`; it has no unit of its own.
