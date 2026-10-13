#!/bin/sh
# One-shot setup for the HCS field logger on a Raspberry Pi (Zero W is enough).
# Copy this file and hcs_logger.py to the Pi, then run:   sudo sh setup_pi_logger.sh
# Afterwards the logger runs as a service, starts on every boot, and writes to /home/<user>/hcs_logs.
# Docs: OASMan_ESP32/docs/hcs-field-data.md
set -e

if [ "$(id -u)" -ne 0 ]; then
    echo "run with: sudo sh $0"
    exit 1
fi

USER_NAME="${SUDO_USER:-pi}"
HOME_DIR="$(getent passwd "$USER_NAME" | cut -d: -f6)"
HERE="$(cd "$(dirname "$0")" && pwd)"

if [ ! -f "$HERE/hcs_logger.py" ]; then
    echo "hcs_logger.py must sit next to this script ($HERE)"
    exit 1
fi

if [ "$HERE/hcs_logger.py" != "$HOME_DIR/hcs_logger.py" ]; then
    install -o "$USER_NAME" -g "$USER_NAME" -m 755 "$HERE/hcs_logger.py" "$HOME_DIR/hcs_logger.py"
fi
install -o "$USER_NAME" -g "$USER_NAME" -d "$HOME_DIR/hcs_logs"
usermod -aG dialout "$USER_NAME"

cat > /etc/systemd/system/hcs-logger.service <<EOF
[Unit]
Description=HCS serial logger (OAS-Man manifold)
After=multi-user.target

[Service]
User=$USER_NAME
ExecStart=/usr/bin/python3 $HOME_DIR/hcs_logger.py --dir $HOME_DIR/hcs_logs
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable --now hcs-logger
sleep 2

echo
echo "service:  $(systemctl is-active hcs-logger)"
echo "port:     $(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo 'none yet (plug in the manifold; the logger waits for it)')"
echo "logs:     $HOME_DIR/hcs_logs/   ->  tail -f $HOME_DIR/hcs_logs/hcs-*.log"
echo "collect:  scp '$USER_NAME@$(hostname).local:hcs_logs/hcs-*.log' ."
