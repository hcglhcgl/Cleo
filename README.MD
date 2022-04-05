# Cleo

## SSH
All commands are used in Linux

### SHH onto Cleo
```bash
ssh local@10.209.232.84
```
- Password: grenen

### SSH folder from Cleo onto personal computer
- Create folder (e.g. Cleo_files)
```bash
sshfs local@192.168.43.38:/ /home/magnus/Cleo_files/
```

### Unmount Folder "Cleo_files"
```bash
sudo umount Cleo_files
```

### SSH in Visual Studio Code
- Open VSC
- Click on green box ('Open a Remote Window') in lower left corne
- Click 'Connect to Host...'
- ssh local@192.168.43.38

## Hotspot
### Change your hotspot to:
- SSID "jul"
- password "12345677"

### Add/change device Cleo should connect to:
- SSH onto Cleo
```bash
sudo nano /etc/wpa_supplicant/wpa_supplicant.conf
```
- Copy another hotspot-device to add/change


## Run script from Visual Studio Code
```bash
cd /Cam_mission/build
make
./mission
```
