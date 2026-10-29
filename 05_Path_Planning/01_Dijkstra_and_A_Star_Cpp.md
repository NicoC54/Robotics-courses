# Dijkstra & A* commentés (C++)

Source : [Dijkstra-A-Project](https://github.com/NicoC54/Dijkstra-A-Project)

## Dijkstra

```cpp
#include <iostream>
#include <vector>
#include <queue>


struct Edge{
    int destination_id;
    double length;
};

struct Vertex {
    double lon{0.0};
    double lat{0.0};
    double x{0.0};
    double y{0.0};
    std::vector<Edge> neighbors;
};


double dijkstra_algorithm(int starting_node, int ending_node, const std::vector<std::vector<Edge>>& graph ){

    const int INF = 10E9; //initialisation des longueurs à l'infini
    std::vector<double> distances(graph.size(), INF); // initialisation du tableau contenant la distance du point dorigine a chaque noeud
    std::vector<int> parents(graph.size(), -1); // initialisation du tableau parent : de quel noeud provient chaque noeud exploré
    using PairElement = std::pair<float,int> ; //simplification du type std::pair<float,int>

    std::priority_queue<PairElement, std::vector<PairElement>, std::greater<PairElement>> priority_queue; // priority queue avec min heap
    distances[starting_node]=0; // initialisation de la distance du noeud de départ
    priority_queue.push({0.0, starting_node}); // exploration du noeud de départ, et donc plus tard dans lalgo de ses voisins prometteurs

    while(!priority_queue.empty()){ // tant qu'il reste des sommets explorables
        auto [current_distance, current_node] = priority_queue.top(); // Le noeud voisin le plus prometteur est choisi comme noeud courant
        priority_queue.pop(); // on supprime ce choix de voisin

        if (current_node == ending_node){ // mathématiquement assure que le résultat est le minimum, car on prend la valeur la plus petite et les poids sont positifs
            return current_distance;
        }

        if (current_distance > distances[current_node]){ // au cas ou on aurait trouvé un chemin au debut plus long (qui était prioritaire au début par rapport à un chemin plus court) mais pas traité et qu'entre temps on trouve un raccourci, ce raccourci sera traité en premier, et viendra à nouveau le cas de ce noeud non prioritaire qu'on supprime avec cette condition
            continue;
        }

        for (const auto& edge : graph[current_node]){ //on explore les voisins (edges) du noeud voisin le plus prometteur qui est actuellement choisi comme noeud courant à partir de graph(current_node) qui donne accès a un vecteur de edges : les edges voisins, et on boucle dessus
            double new_distance = edge.length + current_distance; //on calcule la nouvelle distance à chacun de ces voisins par rapport au noeud d'origine,un part un, qui vaut la distance à l'origine du noeud prometteur choisi l.32 + la distance de ce voisin à ce noeud prometteur
            int neighbor_id = edge.destination_id; // on note le numero du noeud voisin qu'on check actuellement
            if(new_distance<distances[neighbor_id]){ // si la nouvelle distance calculée (current_distance + edge.length est inferieure à toute distance qu'on a enregistré entre lorigine et ce nouveau noeud (distances[neighbor_id]) alors on remplace la distance)
                distances[neighbor_id] = new_distance;
                parents[neighbor_id] = current_node; //on met a jour le parent pour le chemin le plus court
                priority_queue.push({new_distance,neighbor_id}); //on push ce noeud + distance dans la priority queue, parmi tous les voisins du noeud le plus prometteur du départ, la priority queue traitera le plus prometteur (cad avec la ditance cumulée minimale à l'origine ) puis on reboucle
            }


            
        }

    }
    return -1.0f;

}
```

## A*

```cpp
#include <iostream>
#include <queue>
#include <vector>
#include <cmath>

struct Edge {
    int destination_id;
    double length;
};

struct Vertex {
    double x{0.0};
    double y{0.0};
    double lon{0.0};
    double lat{0.0};
    std::vector<Edge> neighbors;
};

// 1. Déclarée avant l'algorithme
double heuristique(int starting_node, int ending_node, const std::vector<Vertex>& vertices) { // fonction pour ajouter une estimation de la distance restante à parcourir du start à l'ending node : distance cartésienne
    double dx = vertices[starting_node].x - vertices[ending_node].x; //xstart-xend =dx
    double dy = vertices[starting_node].y - vertices[ending_node].y; //ystart-yend = dy
    return std::hypot(dx, dy); //racine carrée de dx**2 +dy**2
}

// 2. Passage par référence constante : &
double AstarAlgorithm(int starting_node, int ending_node, 
                      const std::vector<std::vector<Edge>>& graph, 
                      const std::vector<Vertex>& vertices) {

    const double INF = 1e9; //constante pour simuler linfini
    std::vector<int> parents(graph.size(), -1); //tableau de parents de taille graphe.size() et initialisé partout à -1 car pas de parents connus pour lisntant et les id de noeuds démarrent à 0 : hypothese
    std::vector<double> g_score(graph.size(), INF); //tableau des distances des noeuds depuis l'origine appelé distances dans dijkstra, il est appelé g_core ici, puisque f(n)= g(n) + h(n)
    
    using PairElement = std::pair<double, int>; // nouveau type plus simple à écrire
    std::priority_queue<PairElement, std::vector<PairElement>, std::greater<PairElement>> priority_queue; // priority queue min_heap : sortira la valeur de double la plus petite avec un top()

    // 3. Initialisation obligatoire du départ
    g_score[starting_node] = 0.0;  // distance au noeud de départ nulle

    double f_initial_score = /*g_score[starting_node] = 0 +*/ heuristique(starting_node, ending_node, vertices); //initialisation du f_score f(n) = g(n) + h(n) comme  g_score[starting_node] = 0 on ne le met pas
    priority_queue.push({f_initial_score, starting_node}); //attention on a f_initial = heuristique car la distance cumulée au noeud de départ est 0, on initialise lalgo

    while (!priority_queue.empty()) { //tant qu'on a des noeuds à explorer
        auto [f_current_score, current_node] = priority_queue.top(); //on traite le noeud avec le f_score le plus petit en premier puis
        priority_queue.pop(); //on le supprime

        if (current_node == ending_node) { //si le noeuf atteint est celui qu'on voulait atteindre : on retourne g_score la distance reellle depuis le depart
            return g_score[current_node];
        }

        double current_g_score = f_current_score - heuristique(current_node, ending_node, vertices); //f(n) = g(n) + h(n) d'ou g(n) = f(n) - h(n)

        if (current_g_score > g_score[current_node]) { //si on a trouvé un raccourci après avoir inséré un f_score, pour un même noeud, plus élevé dans un premier temps
            continue;
        }

        for (const auto& edge : graph[current_node]) { // on parcourt toutes les arretes du noeud prometteur exploré actuellement, qui mènent à dautres noeuds
            double new_g_score = current_g_score + edge.length; //calcul de la distance reelle pour le noeud qui mène à l'arrete en cours dexplroation
            int neighbor_node_id = edge.destination_id; // id du noeud de l'arrete qu'on traite

            if (new_g_score < g_score[neighbor_node_id]) { //meme logique que dijkstra, si on a trouvé une distance reelle plus petite ici a travers l'arrete et noeud qu'on est en train d'explorer, on met à jour le tableau g_score et parents
                g_score[neighbor_node_id] = new_g_score;
                parents[neighbor_node_id] = current_node;
                double new_f_score = new_g_score + heuristique(neighbor_node_id, ending_node, vertices); //calcul du nouveau f_score = g + h associé à ce noeud voisin, il sera traité en 1er ou non en fonction de si son f_score le plus bas ou non 
                priority_queue.push({new_f_score, neighbor_node_id}); //push du score
            }
        }
    }

    return -1.0; //en cas derreur rencontrée dans le graphe : notamment si on a plus rien dans la priority queue sans etre arrivé au noeud final (passage bloqué/ mauvais noeud/ETC)
}
```
