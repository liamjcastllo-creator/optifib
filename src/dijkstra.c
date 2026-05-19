/*
 * Implementación del algoritmo de Dijkstra con Reporte de Viabilidad y Rastreo Inverso
 */

#include <stdio.h>
#include <float.h>
#include <optifib/graph.h>
#include <optifib/priority_queue.h>
#include <optifib/dijkstra.h>
#include <stdlib.h>

// [NUEVO] Función auxiliar para el Rastreo Inverso (va hacia atrás usando los predecesores)
void printPath(int current_node_id, Graph* graph) {
    if (current_node_id == -1) return;

    // Buscar el nodo actual en el grafo para saber su predecesor y descripción
    Node* n = NULL;
    for (int i = 0; i < graph->V; i++) {
        if (graph->arr[i].nodes->id == current_node_id) {
            n = graph->arr[i].nodes;
            break;
        }
    }

    if (n != NULL) {
        // Llamada recursiva: primero procesa el predecesor para que se imprima de OLT -> Destino
        printPath(n->previous_node_id, graph);
        
        // Imprime el nodo actual. Si tiene predecesor, ponemos una flecha antes
        if (n->previous_node_id != -1) {
            printf(" -> ");
        }
        printf("%s [%d]", n->description, n->id);
    }
}

// [NUEVO] Función auxiliar para evaluar la potencia contra el umbral de -28 dBm
void emitirReporteViabilidad(double perdida_total) {
    const double POTENCIA_OLT = 2.0;      // Potencia estándar de salida en dBm
    const double UMBRAL_CRITICO = -28.0;  // Límite establecido en la propuesta
    
    double potencia_final = POTENCIA_OLT - perdida_total;

    printf("\n============================================\n");
    printf("       REPORTE DE VIABILIDAD OPTICA         \n");
    printf("============================================\n");
    printf("Potencia de Salida (OLT):  %.2f dBm\n", POTENCIA_OLT);
    printf("Perdida Total Calculada:   %.2f dB\n", perdida_total);
    printf("Potencia en el Destino:    %.2f dBm\n", potencia_final);
    printf("--------------------------------------------\n");

    if (potencia_final >= UMBRAL_CRITICO) {
        printf("ESTADO: [ VIABLE / OPERACIONAL ]\n");
        printf("Nota: La senal cumple con los estandares.\n");
    } else {
        printf("ESTADO: [ !!! SENAL CRITICA !!! ]\n");
        printf("ALERTA: La potencia es menor a -28 dBm.\n");
        printf("Accion: Se requiere rediseno o amplificacion.\n");
    }
    printf("============================================\n");
}

// Tu función principal Dijkstra modificada
void dijkstra(Graph* graph, int src_id) {
    if (graph == NULL || graph->V == 0) return;

    // Inicializar distancias y estados
    for (int i = 0; i < graph->V; i++) {
        Node* n = graph->arr[i].nodes;
        n->min_accumulated_loss = DBL_MAX;
        n->visited = 0;
        n->previous_node_id = -1;
    }

    Node* srcNode = nodeFind(graph, src_id);
    if (srcNode == NULL) {
        printf("Error: Nodo origen %d no encontrado.\n", src_id);
        return;
    }

    srcNode->min_accumulated_loss = 0; // OLT no suele tener perdida inicial en el origen

    MinHeap* minHeap = createMinHeap(graph->V);
    for (int i = 0; i < graph->V; i++) {
        minHeap->array[i] = graph->arr[i].nodes;
        minHeap->pos[graph->arr[i].nodes->id] = i;
    }
    minHeap->size = graph->V;

    // Mover el origen al inicio del heap
    decreaseKey(minHeap, src_id, 0.0);

    while (minHeap->size != 0) {
        Node* uNode = extractMin(minHeap);
        uNode->visited = 1;

        if (uNode->min_accumulated_loss == DBL_MAX) break;

        Edge* edge = uNode->adj_list;
        while (edge != NULL) {
            Node* vNode = nodeFind(graph, edge->target_id);
            if (vNode != NULL && !vNode->visited) {
                double weight = edge->link_loss_db + vNode->intrinsic_loss_db;
                if (uNode->min_accumulated_loss + weight < vNode->min_accumulated_loss) {
                    vNode->min_accumulated_loss = uNode->min_accumulated_loss + weight;
                    vNode->previous_node_id = uNode->id;
                    decreaseKey(minHeap, vNode->id, vNode->min_accumulated_loss);
                }
            }
            edge = edge->next;
        }
    }

    // Output de resultados técnicos (Tu tabla original)
    printf("\n+------+---------------------------+----------------+------------+\n");
    printf("| ID   | Descripcion               | Perdida (dB)   | Predecesor |\n");
    printf("+------+---------------------------+----------------+------------+\n");
    for (int i = 0; i < graph->V; i++) {
        Node* n = graph->arr[i].nodes;
        printf("| %-4d | %-25s | ", n->id, n->description);
        if (n->min_accumulated_loss == DBL_MAX) printf("%-14s | ", "INF");
        else printf("%-14.2f | ", n->min_accumulated_loss);
        
        if (n->previous_node_id == -1) printf("%-10s |\n", "N/A");
        else printf("%-10d |\n", n->previous_node_id);
    }
    printf("+------+---------------------------+----------------+------------+\n");

    // [NUEVO] Preguntar al usuario qué nodo específico quiere auditar para trazar su ruta
    int dest_id;
    printf("\nIngrese el ID del nodo destino para analizar la ruta y viabilidad: ");
    if (scanf("%d", &dest_id) == 1) {
        Node* destNode = nodeFind(graph, dest_id);
        if (destNode == NULL) {
            printf("Error: El nodo destino %d no existe.\n", dest_id);
        } else if (destNode->min_accumulated_loss == DBL_MAX) {
            printf("Alerta: El nodo destino %d es inaccesible desde la OLT.\n", dest_id);
        } else {
            // Ejecutar el Rastreo Inverso impreso en orden correcto
            printf("\n>>> RECONSTRUCCION DE RUTA OPTIMA:\n");
            printPath(dest_id, graph);
            printf("\n");

            // Ejecutar la validación de los -28 dBm
            emitirReporteViabilidad(destNode->min_accumulated_loss);
        }
    }

    freeMinHeap(minHeap);
}
