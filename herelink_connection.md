# Setting up routes
On the raspi (temporary):
```
sudo ip addr add 192.168.144.2/24 dev eth0
sudo ip route add 192.168.43.0/24 via 192.168.144.11
```

On the raspi (persistance):
```
nmcli connection modify netplan-eth0 \
  ipv4.method manual \
  ipv4.addresses 192.168.144.2/24 \
  ipv4.dns 8.8.8.8 \
  ipv4.routes "192.168.43.0/24 192.168.144.11" \
  ipv6.method ignore
```
Then run:
```
nmcli connection up netplan-eth0
```

On the ground station machine:
```
sudo ip route add 192.168.144.0/24 via 192.168.43.1
```

# Disable connectivity check on eth0 
## Write a dispatcher script
Create a file ```/etc/NetworkManager/dispatcher.d/99-disable-connectivity-eth0``` with:
```
#!/bin/bash
IF="$1"
ACTION="$2"
CONF_DIR=/etc/NetworkManager/conf.d
CONF_FILE=99-disable-connectivity.conf

case "$IF/$ACTION" in
  eth0/up)
    # drop in the global disable file
    cat > $CONF_DIR/$CONF_FILE <<EOF
[connectivity]
enabled=false
EOF
    ;;
  eth0/down)
    # remove it when eth0 goes away
    rm -f $CONF_DIR/$CONF_FILE
    ;;
  *)
    exit 0
    ;;
esac

# reload NM’s config so it picks up the change immediately
nmcli general reload
```

Then make it executable:
```
sudo chmod +x /etc/NetworkManager/dispatcher.d/99-disable-connectivity-eth0
```
What happens now is:

On eth0 up, the script writes your disable-connectivity snippet and reloads NM.

On eth0 down, it removes that snippet and reloads NM, restoring captive-portal checks for any other links (e.g. your Wi-Fi).

# Usage

To ping run:
```
ping 192.168.144.2
```

To ssh run:
```
ssh eashan@192.168.144.2
```