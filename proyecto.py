import heapq
from collections import Counter
import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class Grafo:
    def __init__(self):
        self.vertices = set()
        self.aristas = {}

    def agregar_vertice(self, vertice):
        self.vertices.add(vertice)

    def agregar_arista(self, desde, hacia, peso):
        if desde not in self.aristas:
            self.aristas[desde] = []
        self.aristas[desde].append((hacia, peso))

    def dijkstra(grafo, inicio):
        distancias = {vertice: float('inf') for vertice in grafo.vertices}
        distancias[inicio] = 0
        cola = [(0, inicio)]
        while cola:
            distancia_actual, vertice_actual = heapq.heappop(cola)
            if distancia_actual > distancias[vertice_actual]:
                continue
            for vecino, peso in grafo.aristas.get(vertice_actual, []):
                distancia_nueva = distancia_actual + peso
                if distancia_nueva < distancias[vecino]:
                    distancias[vecino] = distancia_nueva
                    heapq.heappush(cola, (distancia_nueva, vecino))
        return distancias

def load_balancer_monte_carlo(servidores, mensajes, iteraciones):
    # Inicializar una lista con los servidores, sus cargas, capacidades, anchos de banda y latencias
    servidores_info = []
    for servidor, capacidad in conexiones.items():
        capacidad_almacenamiento = capacidades_almacenamiento[servidor]
        capacidad_almacenamiento_mb = capacidad_almacenamiento * 1024 * 1024
        ancho_banda = anchos_banda[servidor]
        latencia = latencias[servidor]
        servidores_info.append((servidor, 0, capacidad_almacenamiento_mb, ancho_banda, latencia))

    mejor_asignacion = []
    mejor_carga = float('inf')

    # Verificar si hay suficiente capacidad de almacenamiento
    capacidad_total = sum(info[2] for info in servidores_info)
    tamano_total_mensajes = sum(int(mensaje.strip()) for mensaje in mensajes)

    # Realizar una simulación aleatoria
    carga_actual = [0] * len(servidores_info)
    asignacion_actual = []
    for mensaje in mensajes:
        tamano_mensaje = int(mensaje.strip())
        servidores_disponibles = sorted([(i, info) for i, info in enumerate(servidores_info) if info[2] >= (carga_actual[i] + 1) * tamano_mensaje], key=lambda x: (-x[1][3], x[1][4], -conexiones.get(x[1][0], 0)))
        if servidores_disponibles:
            mejor_servidor = servidores_disponibles[0][0]
            carga_actual[mejor_servidor] += 1
            asignacion_actual.append(servidores_info[mejor_servidor][0])

    # Evaluar la función objetivo (en este caso, la carga máxima)
    carga_maxima = max(carga_actual)

    # Actualizar la mejor asignación
    mejor_carga = carga_maxima
    mejor_asignacion = asignacion_actual

    # Imprimir la mejor asignación encontrada
    print("Mejor asignación encontrada:")
    asignacion_final = Counter(mejor_asignacion)
    for servidor, carga in asignacion_final.items():
        print(f"{servidor}: {carga} mensajes")

    # Imprimir el número de mensajes entregados y no entregados
    mensajes_entregados = sum(asignacion_final.values())
    mensajes_no_entregados = len(mensajes) - mensajes_entregados
    print(f"Mensajes entregados: {mensajes_entregados}")
    print(f"Mensajes no entregados: {mensajes_no_entregados}")

    if tamano_total_mensajes > capacidad_total:
        print("No hay suficiente capacidad de almacenamiento en los servidores.")

    return mejor_asignacion

# Definir la topología del grafo
grafo = Grafo()
grafo.agregar_vertice("Router 1")
for i in range(1, 6):
    grafo.agregar_vertice(f"Servidor {i}")

# Conexiones desde el router a los servidores con sus capacidades
conexiones = {
    "Servidor 1": 20,
    "Servidor 2": 15,
    "Servidor 3": 25,
    "Servidor 4": 10,
    "Servidor 5": 30
}

for servidor, capacidad in conexiones.items():
    grafo.agregar_arista("Router 1", servidor, capacidad)

# Capacidades de almacenamiento de los servidores (en terabytes)
capacidades_almacenamiento = {
    "Servidor 1": 8,
    "Servidor 2": 6,
    "Servidor 3": 10,
    "Servidor 4": 5,
    "Servidor 5": 7
}

# Anchos de banda de los servidores (valores arbitrarios para el ejemplo)
anchos_banda = {
    "Servidor 1": 100,
    "Servidor 2": 80,
    "Servidor 3": 120,
    "Servidor 4": 90,
    "Servidor 5": 110
}

# Latencias de los servidores (valores arbitrarios para el ejemplo)
latencias = {
    "Servidor 1": 5,
    "Servidor 2": 10,
    "Servidor 3": 8,
    "Servidor 4": 15,
    "Servidor 5": 7
}

# Leer los mensajes desde el archivo
with open("mensajes.txt", 'r') as file:
    mensajes = file.readlines()

def visualizar_grafo(grafo, asignacion):
    G = nx.Graph()

    # Agregar nodos
    G.add_nodes_from(grafo.vertices)

    # Agregar aristas
    for vertice, aristas in grafo.aristas.items():
        for arista in aristas:
            G.add_edge(vertice, arista[0], weight=arista[1])

    # Colores para los servidores en la asignación
    colores = ['blue', 'green', 'red', 'cyan', 'magenta', 'yellow']

    # Dibujar el grafo
    pos = nx.spring_layout(G, k=0.15, iterations=50)  # Posiciones de los nodos
    fig, ax = plt.subplots(figsize=(12, 8))  # Aumentar el tamaño de la figura
    nx.draw(G, pos, with_labels=True, node_color='lightblue', node_size=2000, font_size=12, font_weight='bold')  # Aumentar el tamaño de la fuente

    # Lista para almacenar las líneas que representarán los mensajes
    lineas_mensajes = []

    # Función auxiliar para actualizar la animación
    def animar(frame):
        for linea in lineas_mensajes:
            linea.remove()
        lineas_mensajes.clear()

        for i, servidor in enumerate(asignacion[:frame]):
            origen, destino = "Router 1", servidor
            if destino in pos:
                linea, = ax.plot([], [], '-', lw=2, color=colores[i % len(colores)])
                linea.set_data([pos[origen][0], pos[destino][0]], [pos[origen][1], pos[destino][1]])
                lineas_mensajes.append(linea)

        return lineas_mensajes

    # Configurar la animación
    ax.set_title("Topología del Grafo y Flujo de Mensajes", fontsize=16, fontweight='bold')  # Aumentar el tamaño del título
    ani = animation.FuncAnimation(fig, animar, frames=len(asignacion), interval=500, blit=True, repeat=False)

    plt.tight_layout()  # Ajustar el layout para evitar recortes
    plt.show()

# Realizar la simulación de Monte Carlo
mejor_asignacion = load_balancer_monte_carlo(conexiones.keys(), mensajes, iteraciones=1)

# Visualizar el grafo y la animación
visualizar_grafo(grafo, mejor_asignacion)