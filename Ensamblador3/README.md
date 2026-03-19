
**Asignatura:** Robótica  
**Facultad:** Ingeniería Mecatrónica  
**Equipo:** Samantha ZOta, Nicolas Acosta y Juan David Torres

---

## 📋 Descripción General
Este repositorio centraliza el desarrollo de los avances respecto al ensamblador 3 tanto calculos, simulaciones, planos y diagrama de bloques 

---

## 📂 Guía de Carpetas

### 🏗️ [CAD](./CAD)
Repositorio de diseño mecánico y modelado 3D.
* **Uso:** Almacenamiento de piezas (.SLDPRT), ensambles (.SLDASM) y archivos de intercambio (.STEP).
* **Software:** SolidWorks / Inventor.

### ⚡ [Controldemotores](./Controldemotores)
Programación de hardware y lógica de bajo nivel.
* **Uso:** Scripts para controladores de motores, configuración de drivers y manejo de señales PWM.
* **Hardware:** Arduino, ESP32, Drivers L298N/A4988.

### 📊 [Diagramadebloques](./Diagramadebloques)
Documentación arquitectónica del sistema.
* **Uso:** Reconocimiento de unidades tanto logicas como hardware

### 📐 [Interfazcinematicadirectaeinversa](./Interfazcinematicadirectaeinversa)
Núcleo matemático del movimiento del robot.
* **Uso:** Implementación de matrices de Denavit-Hartenberg y algoritmos para el cálculo de posicion y angulos en el espacio de trabajo.

### 🧪 [MATLAB](./MATLAB)
Simulación, análisis y procesamiento de señales.
* **Uso:** Modelos de Simulink, scripts de análisis de datos y validación de algoritmos cinemáticos antes de la implementación física.

### 🖥️ Centros de Mando y Monitoreo (ROS)
Todo lo que se realiza en ROS tanto simulaciones con Gazebo y RViz

---
*Este repositorio es propiedad académica del grupo de Robótica - Universidad Militar Nueva Granada.*