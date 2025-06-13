/*
 * filter.h
 *
 *  Created on: Jun 10, 2025
 *      Author: FeruMaga
 */

#ifndef INC_FILTER_H_
#define INC_FILTER_H_

// Estrutura para armazenar os coeficientes de um filtro Butterworth de 2ª ordem.
typedef struct {
    double B0, B1, B2;
    double A1, A2;
} FilterCoefficients_t;

// Estrutura para armazenar as variáveis de estado de um filtro Butterworth de 2ª ordem.
typedef struct {
    double x1, x2; // Amostras de entrada anteriores (x[n-1], x[n-2])
    double y1, y2; // Amostras de saída anteriores (y[n-1], y[n-2])
} Butter2State_t;

// Protótipos das funções do filtro.


// Inicializa o estado de um filtro Butterworth de 2ª ordem.
void butter2_init(Butter2State_t* s);

// Aplica o filtro Butterworth de 2ª ordem a uma nova amostra.
double butter2_apply(Butter2State_t* s, double x, const FilterCoefficients_t* coeffs);

#endif /* INC_FILTER_H_ */
#endif /* INC_FILTER_H_ */
