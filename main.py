import networkx as nx

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
        for vizinho in grafo.neighbors(v):
            if vizinho not in visitados:
                caminho.append(vizinho)
                if dfs_visit(vizinho):
                    return True
                caminho.pop()
        return False
    
    dfs_visit(ponto_inicial)
    return caminho

def astar(grafo, ponto_inicial, ponto_final, heuristica):
    heuristica = nx.get_node_attributes(grafo, 'heuristica')
    visitados = set()
    fronteira = [(0 + heuristica[ponto_inicial], ponto_inicial, [ponto_inicial])]
    
    while fronteira:
        fronteira.sort(reverse=True)  # Ordena por f(n) = g(n) + h(n)
        _, atual, caminho = fronteira.pop()
        if atual == ponto_final:
            return caminho
        if atual not in visitados:
            visitados.add(atual)
            print("Avaliando ponto:", atual)
            for vizinho in grafo.neighbors(atual):
                if vizinho not in visitados:
                    novo_caminho = caminho + [vizinho]
                    custo = len(novo_caminho) - 1 + heuristica[vizinho]  # g(n) + h(n)
                    fronteira.append((custo, vizinho, novo_caminho))
    
    return None

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
    caminho_astar = astar(grafo, ponto_inicial, ponto_final, heuristica)
    print("Caminho encontrado (A*):", caminho_astar)
else:
    print("Opção inválida.")
