import networkx as nx
import matplotlib.pyplot as plt

# Função para ler o arquivo e criar o grafo
def ler_arquivo(nome_arquivo):
    grafo = nx.Graph()
    
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
    
    return grafo, ponto_inicial, ponto_final

# Nome do arquivo de entrada
nome_arquivo = "grafo2.txt"

# Ler o arquivo e criar o grafo
grafo, _, _ = ler_arquivo(nome_arquivo)

# Desenhar o grafo
pos = nx.spring_layout(grafo)  # Posicionamento dos nós
nx.draw(grafo, pos, with_labels=True, node_size=700, node_color='skyblue')  # Desenhar os nós
labels = nx.get_edge_attributes(grafo, 'weight')
nx.draw_networkx_edge_labels(grafo, pos, edge_labels=labels)  # Desenhar os rótulos das arestas
plt.show()  # Mostrar o gráfico
