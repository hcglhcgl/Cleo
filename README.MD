# Cleo

## SSH
All commands are used in Linux

SHH onto Cleo
- Terminal:
```bash
ssh local@10.209.232.84
```
- Password: grenen

SSH folder from Cleo onto personal computer
- Create folder (e.g. Cleo_files)
- Terminal:
```bash
sshfs local@192.168.43.38:/ /home/magnus/Cleo_files/
```

Unmount Folder "Cleo_files"
- Terminal:
```bash
sudo umount Cleo_files
```

### SSH in Visual Studio Code
- Open VSC
- Click on green box ('Open a Remote Window') in lower left corne
- Click 'Connect to Host...'
- ssh local@192.168.43.38

## Hotspot

Change your hotspost to this SSID and make the password:
- SSID "jul"
- password "12345677"


## Run script from Visual Studio Code
- Cd into build on Cleo
```bash
make
./mission
```
