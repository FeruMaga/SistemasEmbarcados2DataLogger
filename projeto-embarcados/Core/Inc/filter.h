/*
 * filter.h
 *
 *  Created on: Jun 12, 2025
 *      Author: FeruMaga
 */

#ifndef INC_FILTER_H_
#define INC_FILTER_H_

#define FILTER_ALPHA_TENSAO       0.1f
#define FILTER_ALPHA_CORRENTE     0.05f
#define FILTER_ALPHA_TEMPERATURA  0.2f
#define FILTER_ALPHA_MASSA        0.02f

float Filter_Lowpass(float oldValue, float newValue, float alpha);

#endif /* INC_FILTER_H_ */



#endif /* INC_FILTER_H_ */
