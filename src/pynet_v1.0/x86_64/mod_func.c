#include <stdio.h>
#include "hocdec.h"
extern int nrnmpi_myid;
extern int nrn_nobanner_;

extern void _ITGHK_reg(void);
extern void _Ih_reg(void);
extern void _hh2_reg(void);

void modl_reg(){
  if (!nrn_nobanner_) if (nrnmpi_myid < 1) {
    fprintf(stderr, "Additional mechanisms from files\n");

    fprintf(stderr," ITGHK.mod");
    fprintf(stderr," Ih.mod");
    fprintf(stderr," hh2.mod");
    fprintf(stderr, "\n");
  }
  _ITGHK_reg();
  _Ih_reg();
  _hh2_reg();
}
