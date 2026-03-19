import tkinter as tk
from tkinter import messagebox, ttk
import math
import serial
import time
import numpy as np

COLOR_FONDO = "#121212"      o
COLOR_TARJETA = "#1E1E1E"    
COLOR_TEXTO = "#FFFFFF"      
COLOR_ACCENTO = "#3700B3"    


# CONFIGURACIÓN SERIAL 

PUERTO = 'COM3' 
BAUDIOS = 115200
try:
    esp32 = serial.Serial(PUERTO, BAUDIOS, timeout=1)
    time.sleep(2)
    print("Conexión serial exitosa.")
except:
    esp32 = None
    print("Modo simulación activo.")

def enviar_angulos_esp32(q1_deg, q2_deg, q3_deg):
    if esp32 and esp32.is_open:
        q_rad = [math.radians(x) for x in [q1_deg, q2_deg, q3_deg]]
        mensaje = f"{q_rad[0]:.4f},{q_rad[1]:.4f},{q_rad[2]:.4f}\n"
        esp32.write(mensaje.encode('utf-8'))

# PARÁMETROS Y MATEMÁTICA
d1, a1, a2, a3 = 15.0, 0.0, 13.0, 13.0

def matriz_DH(theta, d, a, alpha):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0.0,            np.sin(alpha),                np.cos(alpha),               d],
        [0.0,            0.0,                          0.0,                         1.0]
    ])


# INTERFAZ GRÁFICA 

ventana = tk.Tk()
ventana.title("Robot 3 GDL - Dark Mode")
ventana.geometry("400x680")
ventana.configure(bg=COLOR_FONDO)

estilo = ttk.Style()
estilo.theme_use('default')


estilo.configure("TFrame", background=COLOR_FONDO)

estilo.configure("TLabel", background=COLOR_FONDO, foreground=COLOR_TEXTO, font=("Consolas", 10))

estilo.configure("TButton", padding=5, font=("Consolas", 10, "bold"))

estilo.configure("TRadiobutton", background=COLOR_FONDO, foreground=COLOR_TEXTO, font=("Consolas", 10))


main_frame = ttk.Frame(ventana, padding="20")
main_frame.pack(fill="both", expand=True)

titulo = tk.Label(main_frame, text="CONTROL DE ROBOT", bg=COLOR_FONDO, fg=COLOR_TEXTO, font=("Consolas", 14, "bold"))
titulo.pack(pady=10)

ttk.Label(main_frame, text=">> CINEMÁTICA DIRECTA", font=("Consolas", 11, "bold")).pack(pady=5, anchor="w")
f1 = ttk.Frame(main_frame)
f1.pack(pady=5)
entry_q1 = tk.Entry(f1, width=8, bg=COLOR_TARJETA, fg="white", insertbackground="white"); entry_q1.grid(row=0, column=0, padx=5)
entry_q2 = tk.Entry(f1, width=8, bg=COLOR_TARJETA, fg="white", insertbackground="white"); entry_q2.grid(row=0, column=1, padx=5)
entry_q3 = tk.Entry(f1, width=8, bg=COLOR_TARJETA, fg="white", insertbackground="white"); entry_q3.grid(row=0, column=2, padx=5)

def calcular_cd():
    try:
        q = [float(entry_q1.get()), float(entry_q2.get()), float(entry_q3.get())]
        rads = [math.radians(x) for x in q]
        T03 = matriz_DH(rads[0], d1, a1, math.pi/2) @ matriz_DH(rads[1], 0, a2, 0) @ matriz_DH(rads[2], 0, a3, 0)
        lbl_res_cd.config(text=f"POSICIÓN: X:{T03[0,3]:.2} | Y:{T03[1,3]:.2} | Z:{T03[2,3]:.2}")
        enviar_angulos_esp32(*q)
    except: messagebox.showerror("Error", "Datos inválidos")

ttk.Button(main_frame, text="EJECUTAR CD", command=calcular_cd).pack(pady=10)
lbl_res_cd = ttk.Label(main_frame, text="X: 0 | Y: 0 | Z: 0", foreground="#FFFFFF")
lbl_res_cd.pack()


tk.Frame(main_frame, height=1, bg=COLOR_TEXTO).pack(fill="x", pady=25)

ttk.Label(main_frame, text=">> CINEMÁTICA INVERSA", font=("Consolas", 11, "bold")).pack(pady=5, anchor="w")
f2 = ttk.Frame(main_frame)
f2.pack(pady=5)
ent_x = tk.Entry(f2, width=8, bg=COLOR_TARJETA, fg="white", insertbackground="white"); ent_x.grid(row=0, column=0, padx=5)
ent_y = tk.Entry(f2, width=8, bg=COLOR_TARJETA, fg="white", insertbackground="white"); ent_y.grid(row=0, column=1, padx=5)
ent_z = tk.Entry(f2, width=8, bg=COLOR_TARJETA, fg="white", insertbackground="white"); ent_z.grid(row=0, column=2, padx=5)

sel_codo = tk.StringVar(value="abajo")
ttk.Radiobutton(main_frame, text="Codo Arriba", variable=sel_codo, value="arriba").pack()
ttk.Radiobutton(main_frame, text="Codo Abajo", variable=sel_codo, value="abajo").pack()

def calcular_ci():
    try:
        px, py, pz = float(ent_x.get()), float(ent_y.get()), float(ent_z.get())
        pxp = math.sqrt(px**2 + py**2)
        r = math.sqrt((pz - d1)**2 + (pxp - a1)**2)
        cb, cg = (r**2+a2**2-a3**2)/(2*a2*r), (a2**2+a3**2-r**2)/(2*a2*a3)
        if not (-1 <= cb <= 1) or not (-1 <= cg <= 1): messagebox.showwarning("Rango", "Inalcanzable"); return
        beta, gamma, alpha, th1 = math.acos(cb), math.acos(cg), math.atan2(pz-d1, pxp-a1), math.atan2(py, px)
        th2, th3 = (alpha-beta, math.pi-gamma) if sel_codo.get()=="arriba" else (alpha+beta, gamma-math.pi)
        deg = [math.degrees(th1), math.degrees(th2), math.degrees(th3)]
        lbl_res_ci.config(text=f"q1:{deg[0]:.2f}° | q2:{deg[1]:.2f}° | q3:{deg[2]:.2f}°")
        enviar_angulos_esp32(*deg)
    except: messagebox.showerror("Error", "Datos inválidos")

ttk.Button(main_frame, text="EJECUTAR CI", command=calcular_ci).pack(pady=10)
lbl_res_ci = ttk.Label(main_frame, text="q1: 0° | q2: 0° | q3: 0°", foreground="#F1F9F3")
lbl_res_ci.pack()

ventana.mainloop()