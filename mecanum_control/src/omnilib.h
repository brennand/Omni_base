
#ifndef OMNIDRIVE_H 
#define OMNIDRIVE_H 


void jac_forward(double *in, double *out);
void jac_inverse(double *in, double *out);
void twist_to_ws(double x, double y, double a, double &fl, double &fr, double &bl, double &br);

double omnidrive_limit(double x, double l);


#endif
