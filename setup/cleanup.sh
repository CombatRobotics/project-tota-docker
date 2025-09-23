#!/usr/bin/env bash
set -euo pipefail

# Paths
REPO_ROOT="${REPO_ROOT:-$HOME/project-tota}"
USER_UNIT_MAIN="$HOME/.config/systemd/user/tota_ui.service"
DROPIN_DIR="$HOME/.config/systemd/user/tota_ui.service.d"
DROPIN_FILE="$DROPIN_DIR/override.conf"

echo "[cleanup] Preparing environment…"

# 1) Chrome wrapper that never touches keyring (one-time)
mkdir -p "$HOME/.local/bin"
cat > "$HOME/.local/bin/chrome-no-keyring" <<'EOF'
#!/usr/bin/env bash
URL="${1:-http://localhost:3000}"
if command -v google-chrome >/dev/null 2>&1; then
  exec google-chrome --password-store=basic --no-first-run --no-default-browser-check --start-fullscreen --new-window "$URL"
elif command -v chromium-browser >/dev/null 2>&1; then
  exec chromium-browser --password-store=basic --no-first-run --no-default-browser-check --start-fullscreen --new-window "$URL"
else
  echo "Chrome/Chromium not found" >&2
  exit 1
fi
EOF
chmod +x "$HOME/.local/bin/chrome-no-keyring"

# 2) Disable GNOME keyring autostart (prevents login password prompts)
mkdir -p "$HOME/.config/autostart"
for f in /etc/xdg/autostart/gnome-keyring-*.desktop; do
  [ -f "$f" ] || continue
  base="$(basename "$f")"
  cp -f "$f" "$HOME/.config/autostart/$base"
  # Turn off in user copy
  if grep -q '^X-GNOME-Autostart-enabled=' "$HOME/.config/autostart/$base"; then
    sed -i 's/^X-GNOME-Autostart-enabled=.*/X-GNOME-Autostart-enabled=false/' "$HOME/.config/autostart/$base"
  else
    echo 'X-GNOME-Autostart-enabled=false' >> "$HOME/.config/autostart/$base"
  fi
  if grep -q '^Hidden=' "$HOME/.config/autostart/$base"; then
    sed -i 's/^Hidden=.*/Hidden=true/' "$HOME/.config/autostart/$base"
  else
    echo 'Hidden=true' >> "$HOME/.config/autostart/$base"
  fi
done
# Optional one-time cleanup of any old locked keyrings
rm -f "$HOME/.local/share/keyrings/"*.keyring "$HOME/.local/share/keyrings/"*.kdbx 2>/dev/null || true

# 3) Ensure user service sees ~/.local/bin and GUI session bus (systemd drop-in)
mkdir -p "$DROPIN_DIR"
cat > "$DROPIN_FILE" <<'EOF'
[Service]
Environment=DISPLAY=:0
Environment=PATH=%h/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin
Environment=XDG_RUNTIME_DIR=/run/user/%U
Environment=DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/%U/bus
EOF

# 4) Reload user systemd and restart the UI service
systemctl --user daemon-reload
# Make sure the unit is registered
if [ ! -f "$USER_UNIT_MAIN" ] && [ -f "$REPO_ROOT/bootup/startup_services/tota_ui.service" ]; then
  systemctl --user link "$REPO_ROOT/bootup/startup_services/tota_ui.service" || true
fi
systemctl --user enable --now tota_ui.service || true
systemctl --user restart tota_ui.service

# 5) Verify readiness (port 3000)
echo "[cleanup] Waiting for http://localhost:3000 …"
for i in {1..40}; do
  if curl -fsS http://localhost:3000 >/dev/null 2>&1; then
    echo "[cleanup] UI is up. Chrome will auto-launch without keyring prompts."
    exit 0
  fi
  sleep 0.5
done

echo "[cleanup] UI did not become ready. Recent logs:"
journalctl --user -u tota_ui -n 200 --no-pager
exit 1