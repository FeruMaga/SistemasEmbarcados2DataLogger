/*
 * filter.c
 *
 *  Created on: Jun 12, 2025
 *      Author: FeruMaga
 */
#include "filter.h"

// Implementação da função para inicializar o estado do filtro.
void butter2_init(Butter2State_t* s) {
    s->x1 = s->x2 = s->y1 = s->y2 = 0.0;
}

// Implementação da função para aplicar o filtro a uma nova amostra.
void butter2_apply(Butter2State_t* s, double x, const FilterCoefficients_t* coeffs) {
    // A equação de diferença do filtro digital IIR de 2ª ordem.
    // y[n] = B0*x[n] + B1*x[n-1] + B2*x[n-2] - A1*y[n-1] - A2*y[n-2]
    const double y = coeffs->B0 * x + coeffs->B1 * s->x1 + coeffs->B2 * s->x2
                     - coeffs->A1 * s->y1 - coeffs->A2 * s->y2;

    // Atualiza os estados de atraso para a próxima amostra.
    s->x2 = s->x1; // x[n-2] <- x[n-1]
    s->x1 = x;     // x[n-1] <- x[n]
    s->y2 = s->y1; // y[n-2] <- y[n-1]
    s->y1 = y;     // y[n-1] <- y[n]

    return y; // Retorna o novo valor filtrado.
}
