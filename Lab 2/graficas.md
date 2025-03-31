# Comparación de las graficas usando Plot Juggler
## Comportamiento Control Proporcional
![image](https://github.com/user-attachments/assets/67cdb848-0da4-4c5b-834a-b5f4afaff2f9)




## Comportamiento Control Proporcional Derivativo
![image](https://github.com/user-attachments/assets/57dcf560-3ad7-4aeb-8644-ebb36fb1a1f7)



## Comportamiento Control Proporcional Integral Derivativo
![image](https://github.com/user-attachments/assets/ffffdd2b-f6ec-47c4-aaef-61c8bccc5ae9)

# Comparación Control P vs PD vs PID (Objetivo: x=5, y=5, θ=90°)

## 📈 Comportamiento General

| Control | Trayectoria          | Error Posición       | Error Angular    | Velocidad         | Estabilidad       |
|---------|----------------------|----------------------|------------------|-------------------|-------------------|
| **P**   | Oscilaciones         | No converge a 0      | Pequeño offset   | Cambios bruscos   | Inestable         |
| **PD**  | Suave, poco overshoot| Converge con oscil.  | Cero (ligeras osc.) | Perfil suave     | Estable           |
| **PID** | Precisión exacta     | Cero perfecto        | Cero perfecto    | Optimizada        | Muy estable       |

## 🔍 Diferencias Clave

### 1. **Control P (Proporcional)**
- ✅ Rápido inicialmente  
- ❌ Oscila permanentemente  
- 📉 Error estacionario (~5% del objetivo)

### 2. **Control PD (Proporcional-Derivativo)**
- ✅ Amortigua oscilaciones  
- ❌ Pequeño error residual  
- 📊 Mejor equilibrio velocidad/estabilidad

### 3. **Control PID (Proporcional-Integral-Derivativo)**
- ✅ Elimina error estacionario  
- ❌ Más lento en converger  
- ⚙️ Requiere ajuste fino de Ki


