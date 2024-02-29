import networkx as nx

def ler_arquivo(nome_arquivo):
    grafo = nx.Graph()
    ponto_inicial = None
    ponto_final = None
    heuristica = {}
    
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
                heuristica[no] = valor_heuristica
    
    return grafo, ponto_inicial, ponto_final, heuristica

def astar(grafo, ponto_inicial, ponto_final, heuristica):
    visitados = set()
    fronteira = [(0 + heuristica[ponto_inicial], ponto_inicial, [ponto_inicial])]
    
    while fronteira:
        fronteira.sort(reverse=True)
        _, atual, caminho = fronteira.pop()
        if atual == ponto_final:
            return caminho
        if atual not in visitados:
            visitados.add(atual)
            for vizinho in grafo.neighbors(atual):
                if vizinho not in visitados:
                    novo_caminho = caminho + [vizinho]
                    custo = len(novo_caminho) - 1 + heuristica[vizinho]
                    fronteira.append((custo, vizinho, novo_caminho))
    
    return None

nome_arquivo = "grafo2.txt"
grafo, ponto_inicial, ponto_final, heuristica = ler_arquivo(nome_arquivo)

caminho_astar = astar(grafo, ponto_inicial, ponto_final, heuristica)
print("Caminho encontrado (A*):", caminho_astar)
