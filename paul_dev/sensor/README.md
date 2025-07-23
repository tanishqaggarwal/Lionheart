### How to Run

In PIO, hit Upload and Monitor.

### Ethernet Comms

Find the ethernet interface with ifconfig -a and configure it to the following IP.

```
pauljoo@Pauls-MacBook-Air ~ % sudo ifconfig en4 192.168.1.178 netmask 255.255.255.0
```

Verify ethernet connection with

```
pauljoo@Pauls-MacBook-Air ~ % ping 192.168.1.177
```

Send ethernet comms via the following. The firmware is encoded with a static IP. Also here are some sample commands.

```
pauljoo@Pauls-MacBook-Air sensor % nc -u 192.168.1.177 3000
connect
m2_speed 255
m2_speed 0
```
