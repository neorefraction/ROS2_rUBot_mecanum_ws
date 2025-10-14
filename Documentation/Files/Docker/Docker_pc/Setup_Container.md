# Install container

- In a terminal type:
```bash
docker compose -f docker-compose-linux.yml up -d
```

nstruccions (Linux host)
0) (Només una vegada) Permetre l’accés X11 al contenidor

Això deixa entrar processos locals (inclòs Docker) al teu servidor X:

xhost +local:


Si vols ser més restrictiu: xhost si:localuser:root (i/o l’usuari que corri dins del contenidor).

1) Construir la imatge
docker compose -f docker-compose_linux.yaml build

2) Arrencar el contenidor en segon pla
docker compose -f docker-compose_linux.yaml up -d

3) Entrar al contenidor
docker exec -it pc_humble bash

4) Prova ràpida de GUI

Dins del contenidor:

apt-get update && apt-get install -y x11-apps mesa-utils
xclock     # ha d’aparèixer el rellotge X al teu escriptori
# o bé
glxinfo | head

5) Prova ROS GUI (exemples)
rviz2
# o
rqt

Notes importants

Mode host (Linux): el contenidor comparteix IP i ports amb l’host. És ideal per DDS (CycloneDDS/FastDDS) perquè evita NAT i facilita el multicast/unicast. No cal obrir ports a docker; si tens UFW actiu, assegura’t de permetre el trànsit necessari a l’host.

DISPLAY: amb Linux + Xorg, ${DISPLAY} ja vindrà configurat (p. ex. :0). El volum /tmp/.X11-unix és clau.

Wayland (Ubuntu 22.04 sol venir amb Wayland): si tens problemes, inicia sessió amb Xorg (pantalla de login → icona d’engranatge → Ubuntu on Xorg) o usa xhost +local: igualment (moltes apps X funcionen via XWayland).

OpenGL: Si tens errors de GLX, deixa LIBGL_ALWAYS_SOFTWARE=1 i MESA_GL_VERSION_OVERRIDE=2.1. Quan tot funcioni, pots provar d’eliminar-los i mantenir /dev/dri per acceleració via Mesa/VA-API segons el teu hardware/driver.

Windows X server (opcional): si algun dia vols que el contenidor (en una màquina Linux) pinti en un PC Windows amb Xming/X410/etc., canvia DISPLAY per DISPLAY=<IP_windows>:0.0 i no cal muntar /tmp/.X11-unix. Però com que aquest docker-compose_linux.yaml és per Linux host, el flux habitual és pintar al mateix Linux.

CycloneDDS: si ja tens un cyclonedds.xml amb unicast (llistes de peers) o filtres d’IF, munta’l i exporta CYCLONEDDS_URI (veus el bloc comentat del compose).