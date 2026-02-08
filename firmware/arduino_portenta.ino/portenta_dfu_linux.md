# Arduino Portenta H7 – Solución DFU en Linux

Este documento describe cómo solucionar los problemas de permisos USB al cargar código en el **Arduino Portenta H7** desde Linux.

---

## ✅ Solución correcta (5 pasos, copy-paste seguro)

### 1️⃣ Crear el archivo de reglas udev

```bash
sudo nano /etc/udev/rules.d/99-arduino-portenta.rules
```

---

### 2️⃣ Pegar el contenido dentro del archivo

```text
# Arduino Portenta H7 - normal mode
SUBSYSTEM=="usb", ATTR{idVendor}=="2341", MODE="0666"

# Arduino Portenta H7 - DFU mode
SUBSYSTEM=="usb", ATTR{idVendor}=="2341", ATTR{idProduct}=="035b", MODE="0666"
```

**Notas**
- `2341` → Arduino  
- `035b` → Portenta H7 en modo DFU  

---

### 3️⃣ Guardar y salir de nano

- `CTRL + O` → Enter  
- `CTRL + X`

---

### 4️⃣ Recargar reglas udev

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

---

### 5️⃣ Añadir el usuario a los grupos correctos

```bash
sudo usermod -aG dialout,plugdev $USER
```

---

## ⚠️ MUY IMPORTANTE

Después de los pasos anteriores debes:

- Cerrar sesión **o**
- Reiniciar el PC

```bash
reboot
```

---

## 🔁 Modo bootloader (DOS CLICS)

Para cargar el programa en el Portenta:

1. Pulsa **RESET dos veces seguidas (dos clics rápidos)**
2. El LED queda **verde pulsando**
3. Pulsa **Upload** en el Arduino IDE

---

## ✅ Resultado esperado

Durante la carga:

```text
Opening DFU capable USB device...
Download done.
File downloaded successfully
```

---
