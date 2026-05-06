Nosso cenário é de um robô seguidor de linha competitivo utilizando ESP32 dual core, que precisa correr em alta velocidade sem perder leituras da linha, corre de forma autônoma, e tem seus estados controlados via Bluetooth Low Energy (correr, parar, calibração e configurações, a corrida do robô não é controlada mas sim autônoma).

# 🧠 1. O que usar vs o que evitar

## ✅ **Enum FSM + Active Object (leve) → RECOMENDADO**

* Um **AO principal** para controle de estados (BLE → eventos)
* FSM simples (`enum + switch`)
* Loop crítico fora disso

✔ Por quê:

* Zero overhead relevante
* Determinístico
* Fácil de auditar timing

---

## ✅ **Tasks separadas (pipeline) → ESSENCIAL**

Você PRECISA separar:

* leitura de sensores
* controle (PID)
* comunicação (BLE)

Senão:

> BLE vai roubar tempo de CPU → você perde leitura → robô sai da linha

---

## ⚠️ **State Pattern (classes) → usar com cuidado**

Pode funcionar, MAS:

* virtual calls
* indireção
* mais complexidade

👉 Em ESP32 isso não costuma matar performance, mas:

* adiciona jitter
* dificulta debugging em tempo real

✔ Use só se sua FSM crescer muito

---

## ❌ **HSM (hierárquica) → OVERKILL aqui**

Para line follower competitivo:

* poucos estados
* transições simples

👉 HSM adiciona complexidade sem ganho real

---

## ❌ **QP Framework → provavelmente piora seu caso**

QP é excelente, mas:

* adiciona camada de abstração
* mais indireção
* curva de aprendizado

👉 Para:

* drone autônomo complexo → faz sentido
* line follower competitivo → desnecessário

---

## ❌ **ROS → completamente fora do escopo**

* latência alta
* overhead absurdo para ESP32
* não é RT determinístico

---

# 🧠 2. O que REALMENTE importa (competição)

Ordem de prioridade:

1. **Leitura de sensores determinística**
2. **Loop de controle com período fixo (PID)**
3. **Baixo jitter**
4. **Zero bloqueio**
5. **Cache/localidade (core affinity)**

Arquitetura vem depois disso.

---

# ⚙️ 3. Separação ideal por core (ESP32 dual-core)

ESP32:

* Core 0 → sistema (WiFi, BLE stack)
* Core 1 → aplicação (recomendado)

---

## 🎯 Estratégia padrão vencedora

### 🔵 Core 1 (APP CPU) — tempo real

Tudo crítico roda aqui:

### Task 1 — Sensor (ALTA prioridade)

```cpp
void sensorTask(void*) {
    while (true) {
        readLineSensors();  // ADC / GPIO
        xQueueOverwrite(sensorQueue, &data);
    }
}
```

### Task 2 — Controle (PID) (ALTA prioridade)

```cpp
void controlTask(void*) {
    while (true) {
        SensorData data;
        xQueueReceive(sensorQueue, &data, portMAX_DELAY);

        float error = computeError(data);
        float output = pid(error);

        setMotors(output);
    }
}
```

👉 Essas duas são o coração do robô

---

### 🔴 Core 0 — comunicação / baixo impacto

### Task 3 — BLE (BAIXA prioridade)

```cpp
void bleTask(void*) {
    while (true) {
        Event evt = waitBLE();
        xQueueSend(robotQueue, &evt, 0);
    }
}
```

### Task 4 — Active Object (estado)

```cpp
void robotTask(void*) {
    while (true) {
        Event evt;
        xQueueReceive(queue, &evt, portMAX_DELAY);
        handleEvent(evt);
    }
}
```

---

# 🧩 4. Pinando tasks nos cores

Use:

```cpp
xTaskCreatePinnedToCore(
    sensorTask,
    "sensor",
    2048,
    NULL,
    10,          // prioridade alta
    NULL,
    1            // Core 1 (APP)
);
```

```cpp
xTaskCreatePinnedToCore(
    bleTask,
    "ble",
    4096,
    NULL,
    2,           // baixa prioridade
    NULL,
    0            // Core 0
);
```

---

# ⚠️ 5. Erros que fazem robô perder corrida

## ❌ BLE no mesmo core do controle

→ jitter → leitura falha

---

## ❌ `printf` no loop crítico

→ travamentos aleatórios

---

## ❌ fila bloqueante no controle

```cpp
xQueueReceive(..., portMAX_DELAY);
```

→ se travar, você perde o ciclo

---

## ❌ lógica pesada no handler de estado

→ atraso de eventos críticos

---

## ❌ uso de `delay()` em vez de timing fixo

→ controle instável

---

# 🧠 6. Arquitetura ideal (resumo)

```text
                (Core 0)
           ┌──────────────┐
           │    BLE Task  │
           └──────┬───────┘
                  │
                  ▼
           ┌──────────────┐
           │ Active Object│
           │ (FSM leve)   │
           └──────────────┘


                (Core 1)
     ┌────────────────────────────┐
     │ Sensor Task (alta freq)    │
     └──────────┬─────────────────┘
                ▼
     ┌────────────────────────────┐
     │ Control Task (PID loop)    │
     └──────────┬─────────────────┘
                ▼
            Motores
```

---

# 🚀 Insight final (bem direto)

Se você fizer isso aqui, já está à frente de muita gente:

* ✔ Sensor + controle no mesmo core (isolado)
* ✔ BLE separado
* ✔ FSM leve (sem overengineering)
* ✔ Nenhum bloqueio no loop crítico

---

# ⚠️ 7. Pontos CRÍTICOS

## 🚫 Nunca faça no controlLoop:

* printf
* xQueueSend
* malloc
* BLE
* qualquer coisa bloqueante

## ⚠️ Sobre esp_timer em ISR

* use IRAM_ATTR
* funções devem estar em IRAM
* evitar chamadas pesadas

* use IRAM_ATTR
* funções devem estar em IRAM
* evitar chamadas pesadas

---

# 🧠 8. Integração correta FSM ↔ controle

👉 Esse é o ponto mais importante da arquitetura:

## ❌ ERRADO

FSM mandar comandos via fila para controle

→ latência
→ jitter
→ perda de determinismo

## ✅ CORRETO

FSM apenas faz:

```cpp
gControlState.mode = RUNNING;
```


Controle faz:
```cpp
if (mode == RUNNING) { ... }
```

👉 polling atômico → previsível e rápido

---

Short answer: **prefer instantiating Active Objects (objects with state), not “static classes.”**
In embedded C++ that usually means either:

* **one global/static *instance*** (no heap), or
* **a statically allocated instance created at startup**

—not a class full of `static` methods and globals.

---

# 🧠 Why “static class” is usually the wrong tool

When people say *static class*, they typically mean:

```cpp
class Robot {
public:
    static void handleEvent(...);
    static State state;
};
```

### ❌ Problems

### 1. Hidden global state

* Harder to reason about
* Easy to accidentally share across contexts

---

### 2. Breaks Active Object semantics

Active Object = **encapsulated state + queue + thread of execution**

With static:

* no real instance
* no isolation
* harder to run multiple AOs

---

### 3. Harder to test

You can’t easily:

```cpp
Robot r1;
Robot r2;  // impossible with static-only design
```

---

# ✅ Why instantiated Active Objects are better

```cpp
class RobotAO {
public:
    void start();
    void postEvent(const Event& e);

private:
    QueueHandle_t queue;
    RobotState state;
};
```

---

## Benefits

### 1. Proper encapsulation

* state belongs to the object
* no accidental sharing

---

### 2. Matches RTOS model

Each AO =

* one task
* one queue
* one state machine

👉 clean mental model

---

### 3. Scales naturally

Even if today you have one robot controller:

Tomorrow:

* motor controller AO
* sensor AO
* comms AO

---

### 4. Testability

You can run:

```cpp
RobotAO robot;
robot.handleEvent(...);
```

without FreeRTOS if needed

---

# ⚙️ But what about memory constraints?

Good instinct — dynamic allocation is risky in embedded.

## ❌ Avoid

```cpp
auto robot = new RobotAO();  // heap fragmentation risk
```

---

## ✅ Preferred patterns

### 1. Static instance (best of both worlds)

```cpp
static RobotAO robot;
```

✔ no heap
✔ still an object
✔ deterministic

---

### 2. Static allocation with FreeRTOS

Use:

```cpp
xTaskCreateStatic(...)
xQueueCreateStatic(...)
```

👉 zero dynamic memory

---

# 🧠 What about your specific architecture?

## Core 0 (FSM / BLE)

```cpp
static RobotAO robotAO;
```

✔ perfect use case

---

## Core 1 (control loop)

👉 **DO NOT wrap control loop in an Active Object**

That part should be:

* tight
* minimal
* deterministic

Using:

* plain functions
* static/global structs (carefully)

---

# ⚖️ When static *does* make sense

Use `static` for:

### ✅ Hardware drivers (stateless or singleton)

```cpp
class MotorDriver {
public:
    static void setSpeed(int left, int right);
};
```

✔ OK because:

* no internal mutable state (or controlled)
* maps to hardware

---

### ✅ Shared config

```cpp
struct Config {
    static float Kp;
};
```

---

### ⚠️ But not for:

* state machines
* Active Objects
* anything with evolving internal state

---

# 🚀 Recommended pattern (for your robot)

```cpp
// Core 0
static RobotAO robotAO;

// Core 1
// no class, just:
void controlLoop();  // tight ISR/timer-driven
```

---

# 🔥 Key insight

> Active Object is about **owning state and execution context**

A static class:

* removes ownership
* blurs boundaries
* increases coupling

---

# 🧠 Rule of thumb

* If it has **state + behavior over time** → use an **instance**
* If it’s just **utility or hardware access** → static is fine

---

If you want, I can show you a **zero-heap Active Object implementation using only static allocation (queues + tasks)** — that’s the sweet spot for ESP32 in production.
