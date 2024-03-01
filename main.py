import networkx as nx
import heapq

def pause():
    input("Pressione Enter para continuar...")
    print("------------------------------------------------------------------------------")

def ler_arquivo(nome_arquivo):
    grafo = nx.Graph()
    ponto_inicial = None
    ponto_final = None
    heuristica = {}  # Dicionário para armazenar as heurísticas dos nós
    
    with open(nome_arquivo, 'r') as arquivo:
        linhas = arquivo.readlines()
        
        for linha in linhas:
            partes = linha.split('(')
            predicado = partes[0]
            argumentos = partes[1].split(')')[0].split(',')
            
            if predicado == 'ponto_inicial':
                ponto_inicial = argumentos[0]
            elif predicado == 'ponto_final':
                ponto_final = argumentos[0]
            elif predicado == 'pode_ir':
                peso = int(argumentos[2])
                grafo.add_edge(argumentos[0], argumentos[1], weight=peso)
            elif predicado == 'h':
                no = argumentos[0]
                valor_heuristica = int(argumentos[2])
                grafo.nodes[no]['heuristica'] = valor_heuristica
                heuristica[no] = valor_heuristica  # Adiciona a heurística ao dicionário
    
    return grafo, ponto_inicial, ponto_final, heuristica

def dijkstra(grafo, ponto_inicial, ponto_final):
    # Inicializa os custos dos vértices como infinito e o anterior como None
    custos = {v: float('inf') for v in grafo.nodes}
    anterior = {v: None for v in grafo.nodes}
    # Define o custo do ponto inicial como 0
    custos[ponto_inicial] = 0
    # Conjunto de nós visitados
    visitados = set()
    
    while visitados != set(grafo.nodes):
        # Escolhe o nó não visitado com menor custo
        u = min((custo, v) for v, custo in custos.items() if v not in visitados)[1]
        visitados.add(u)
        print("Avaliando ponto:", u)
        
        # Apresenta custos acumulados
        print("Custos acumulados:", custos)
        
        # Apresenta nós visitados até o momento
        print("Nós visitados:", visitados)

        # Apresenta o caminho parcial encontrado
        print("Caminho parcial encontrado:", reconstruct_path(anterior, u))

        pause()
        
        # Atualiza os custos dos vizinhos de u
        for v in grafo.neighbors(u):
            novo_custo = custos[u] + grafo[u][v]['weight']
            if novo_custo < custos[v]:
                custos[v] = novo_custo
                anterior[v] = u
    
    # Constrói o caminho mínimo
    caminho = []
    v = ponto_final
    while v is not None:
        caminho.insert(0, v)
        v = anterior[v]
    
    return caminho

def dfs(grafo, ponto_inicial, ponto_final):
    visitados = set()
    caminho = [ponto_inicial]
    
    def dfs_visit(v):
        nonlocal caminho
        if v == ponto_final:
            caminho.append(v)
            return True
        visitados.add(v)
        print("Avaliando ponto:", v)
        print("Caminho parcial encontrado:", caminho)  # Impressão do caminho parcial
        pause()
        for vizinho in grafo.neighbors(v):
            if vizinho not in visitados:
                caminho.append(vizinho)
                if dfs_visit(vizinho):
                    return True
                caminho.pop()
        return False
    
    dfs_visit(ponto_inicial)
    return caminho

def reconstruct_path(cameFrom, current):
    total_path = [current]
    while current in cameFrom:
        current = cameFrom[current]
        total_path.insert(0, current)
    return total_path

def A_Star(grafo, ponto_inicial, ponto_final, heuristica):
    openSet = [(0 + heuristica[ponto_inicial], ponto_inicial)]
    cameFrom = {}
    gScore = {node: float('inf') for node in grafo.nodes}
    gScore[ponto_inicial] = 0
    fScore = {node: float('inf') for node in grafo.nodes}
    fScore[ponto_inicial] = heuristica[ponto_inicial]

    while openSet:
        _, current = heapq.heappop(openSet)

        print("Visitando nó:", current)
        print("Valor peso + heurística:", fScore[current])
        print("Custos acumulados:", gScore)
        print("Nós na fila de prioridade:", [node[1] for node in openSet])
        print("Heurística do nó:", heuristica[current])
        print("Caminho parcial encontrado:", reconstruct_path(cameFrom, current))
        pause()

        if current == ponto_final:
            return reconstruct_path(cameFrom, current)

        for neighbor in grafo.neighbors(current):
            tentative_gScore = gScore[current] + grafo[current][neighbor]['weight']
            if tentative_gScore < gScore[neighbor]:
                cameFrom[neighbor] = current
                gScore[neighbor] = tentative_gScore
                fScore[neighbor] = tentative_gScore + heuristica[neighbor]
                heapq.heappush(openSet, (fScore[neighbor], neighbor))

    return None  # Failure
    

nome_arquivo = "grafo2.txt"  # Nome do arquivo de entrada
grafo, ponto_inicial, ponto_final, heuristica = ler_arquivo(nome_arquivo)

opcao = input("Escolha o algoritmo a ser executado (dijkstra, DFS ou A*): ")
if opcao == "dijkstra":
    caminho_minimo = dijkstra(grafo, ponto_inicial, ponto_final)
    print("Caminho mínimo (Dijkstra):", caminho_minimo)
elif opcao == "DFS":
    caminho_dfs = dfs(grafo, ponto_inicial, ponto_final)
    print("Caminho encontrado (Busca em Profundidade):", caminho_dfs)
elif opcao == "A*":
    caminho_astar = A_Star(grafo, ponto_inicial, ponto_final, heuristica)
    print("Caminho encontrado (A*):", caminho_astar)
else:
    print("Opção inválida.")
