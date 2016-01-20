/* Created by Language version: 6.2.0 */
/* NOT VECTORIZED */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "scoplib_ansi.h"
#undef PI
#define nil 0
#include "md1redef.h"
#include "section.h"
#include "nrniv_mf.h"
#include "md2redef.h"
 
#if METHOD3
extern int _method3;
#endif

#if !NRNGPU
#undef exp
#define exp hoc_Exp
extern double hoc_Exp(double);
#endif
 
#define _threadargscomma_ /**/
#define _threadargs_ /**/
 
#define _threadargsprotocomma_ /**/
#define _threadargsproto_ /**/
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 static double *_p; static Datum *_ppvar;
 
#define t nrn_threads->_t
#define dt nrn_threads->_dt
#define gmax _p[0]
#define shift _p[1]
#define i _p[2]
#define h_inf _p[3]
#define tau_s _p[4]
#define alpha _p[5]
#define beta _p[6]
#define k1ca _p[7]
#define k3p _p[8]
#define m _p[9]
#define c1 _p[10]
#define o1 _p[11]
#define o2 _p[12]
#define p0 _p[13]
#define p1 _p[14]
#define eh _p[15]
#define Dc1 _p[16]
#define Do1 _p[17]
#define Do2 _p[18]
#define Dp0 _p[19]
#define Dp1 _p[20]
#define cai _p[21]
#define ih _p[22]
#define gh _p[23]
#define tadj _p[24]
#define _g _p[25]
#define _ion_eh	*_ppvar[0]._pval
#define _ion_ih	*_ppvar[1]._pval
#define _ion_dihdv	*_ppvar[2]._pval
#define _ion_cai	*_ppvar[3]._pval
 
#if MAC
#if !defined(v)
#define v _mlhv
#endif
#if !defined(h)
#define h _mlhh
#endif
#endif
 
#if defined(__cplusplus)
extern "C" {
#endif
 static int hoc_nrnpointerindex =  -1;
 /* external NEURON variables */
 extern double celsius;
 /* declaration of user functions */
 static void _hoc_activation(void);
 static void _hoc_evaluate_fct(void);
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_prop_size(int, int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
extern Memb_func* memb_func;
 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _p = _prop->param; _ppvar = _prop->dparam;
 }
 static void _hoc_setdata() {
 Prop *_prop, *hoc_getdata_range(int);
 _prop = hoc_getdata_range(_mechtype);
   _setdata(_prop);
 hoc_retpushx(1.);
}
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 "setdata_htc", _hoc_setdata,
 "activation_htc", _hoc_activation,
 "evaluate_fct_htc", _hoc_evaluate_fct,
 0, 0
};
 /* declare global and static user variables */
#define Pc Pc_htc
 double Pc = 0.01;
#define cac cac_htc
 double cac = 0.002;
#define exptemp exptemp_htc
 double exptemp = 36;
#define ginc ginc_htc
 double ginc = 2;
#define k4 k4_htc
 double k4 = 0.001;
#define k2 k2_htc
 double k2 = 0.0004;
#define nexp nexp_htc
 double nexp = 1;
#define nca nca_htc
 double nca = 4;
#define q10 q10_htc
 double q10 = 3;
#define taum taum_htc
 double taum = 20;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "cac_htc", "mM",
 "k2_htc", "1/ms",
 "k4_htc", "1/ms",
 "taum_htc", "ms",
 "gmax_htc", "mho/cm2",
 "shift_htc", "mV",
 "i_htc", "mA/cm2",
 "tau_s_htc", "ms",
 "alpha_htc", "1/ms",
 "beta_htc", "1/ms",
 "k1ca_htc", "1/ms",
 "k3p_htc", "1/ms",
 0,0
};
 static double c10 = 0;
 static double delta_t = 1;
 static double o20 = 0;
 static double o10 = 0;
 static double p10 = 0;
 static double p00 = 0;
 static double v = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "cac_htc", &cac_htc,
 "k2_htc", &k2_htc,
 "Pc_htc", &Pc_htc,
 "k4_htc", &k4_htc,
 "nca_htc", &nca_htc,
 "nexp_htc", &nexp_htc,
 "ginc_htc", &ginc_htc,
 "taum_htc", &taum_htc,
 "q10_htc", &q10_htc,
 "exptemp_htc", &exptemp_htc,
 0,0
};
 static DoubVec hoc_vdoub[] = {
 0,0,0
};
 static double _sav_indep;
 static void nrn_alloc(Prop*);
static void  nrn_init(_NrnThread*, _Memb_list*, int);
static void nrn_state(_NrnThread*, _Memb_list*, int);
 static void nrn_cur(_NrnThread*, _Memb_list*, int);
static void  nrn_jacob(_NrnThread*, _Memb_list*, int);
 
static int _ode_count(int);
static void _ode_map(int, double**, double**, double*, Datum*, double*, int);
static void _ode_spec(_NrnThread*, _Memb_list*, int);
static void _ode_matsol(_NrnThread*, _Memb_list*, int);
 
#define _cvode_ieq _ppvar[4]._i
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "6.2.0",
"htc",
 "gmax_htc",
 "shift_htc",
 0,
 "i_htc",
 "h_inf_htc",
 "tau_s_htc",
 "alpha_htc",
 "beta_htc",
 "k1ca_htc",
 "k3p_htc",
 "m_htc",
 0,
 "c1_htc",
 "o1_htc",
 "o2_htc",
 "p0_htc",
 "p1_htc",
 0,
 0};
 static Symbol* _h_sym;
 static Symbol* _ca_sym;
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
 	_p = nrn_prop_data_alloc(_mechtype, 26, _prop);
 	/*initialize range parameters*/
 	gmax = 2e-05;
 	shift = 0;
 	_prop->param = _p;
 	_prop->param_size = 26;
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 5, _prop);
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_h_sym);
 nrn_promote(prop_ion, 0, 1);
 	_ppvar[0]._pval = &prop_ion->param[0]; /* eh */
 	_ppvar[1]._pval = &prop_ion->param[3]; /* ih */
 	_ppvar[2]._pval = &prop_ion->param[4]; /* _ion_dihdv */
 prop_ion = need_memb(_ca_sym);
 nrn_promote(prop_ion, 1, 0);
 	_ppvar[3]._pval = &prop_ion->param[1]; /* cai */
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 static void _update_ion_pointer(Datum*);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*f)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, _NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _Ih_reg() {
	int _vectorized = 0;
  _initlists();
 	ion_reg("h", 1.0);
 	ion_reg("ca", -10000.);
 	_h_sym = hoc_lookup("h_ion");
 	_ca_sym = hoc_lookup("ca_ion");
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 0);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_reg(_mechtype, 2, _update_ion_pointer);
  hoc_register_prop_size(_mechtype, 26, 5);
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 htc /home/renan/LASCON2016/Projeto/code/pynet_v1.0/x86_64/Ih.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static char *modelname = "anomalous rectifier channel";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int activation(double, double);
static int evaluate_fct(double, double);
 extern double *_getelm();
 
#define _MATELM1(_row,_col)	*(_getelm(_row + 1, _col + 1))
 
#define _RHS1(_arg) _coef1[_arg + 1]
 static double *_coef1;
 
#define _linmat1  1
 static void* _sparseobj1;
 static void* _cvsparseobj1;
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[5], _dlist1[5]; static double *_temp1;
 static int ihkin();
 
static int ihkin ()
 {_reset=0;
 {
   double b_flux, f_flux, _term; int _i;
 {int _i; double _dt1 = 1.0/dt;
for(_i=2;_i<5;_i++){
  	_RHS1(_i) = -_dt1*(_p[_slist1[_i]] - _p[_dlist1[_i]]);
	_MATELM1(_i, _i) = _dt1;
      
} }
 evaluate_fct ( _threadargscomma_ v , cai ) ;
   /* ~ c1 <-> o1 ( alpha , beta )*/
 f_flux =  alpha * c1 ;
 b_flux =  beta * o1 ;
 _RHS1( 2) -= (f_flux - b_flux);
 _RHS1( 3) += (f_flux - b_flux);
 
 _term =  alpha ;
 _MATELM1( 2 ,2)  += _term;
 _MATELM1( 3 ,2)  -= _term;
 _term =  beta ;
 _MATELM1( 2 ,3)  -= _term;
 _MATELM1( 3 ,3)  += _term;
 /*REACTION*/
  /* ~ p0 <-> p1 ( k1ca , k2 )*/
 f_flux =  k1ca * p0 ;
 b_flux =  k2 * p1 ;
 _RHS1( 4) -= (f_flux - b_flux);
 
 _term =  k1ca ;
 _MATELM1( 4 ,4)  += _term;
 _term =  k2 ;
 _MATELM1( 4 ,1)  -= _term;
 /*REACTION*/
  /* ~ o1 <-> o2 ( k3p , k4 )*/
 f_flux =  k3p * o1 ;
 b_flux =  k4 * o2 ;
 _RHS1( 3) -= (f_flux - b_flux);
 
 _term =  k3p ;
 _MATELM1( 3 ,3)  += _term;
 _term =  k4 ;
 _MATELM1( 3 ,0)  -= _term;
 /*REACTION*/
   /* p0 + p1 = 1.0 */
 _RHS1(1) =  1.0;
 _MATELM1(1, 1) = 1;
 _RHS1(1) -= p1 ;
 _MATELM1(1, 4) = 1;
 _RHS1(1) -= p0 ;
 /*CONSERVATION*/
  /* c1 + o1 + o2 = 1.0 */
 _RHS1(0) =  1.0;
 _MATELM1(0, 0) = 1;
 _RHS1(0) -= o2 ;
 _MATELM1(0, 3) = 1;
 _RHS1(0) -= o1 ;
 _MATELM1(0, 2) = 1;
 _RHS1(0) -= c1 ;
 /*CONSERVATION*/
   } return _reset;
 }
 
static int  evaluate_fct (  double _lv , double _lcai ) {
   
/*VERBATIM*/
cai = _ion_cai;
 h_inf = 1.0 / ( 1.0 + exp ( ( _lv + 75.0 - shift ) / 5.5 ) ) ;
   tau_s = ( taum + 1000.0 / ( exp ( ( _lv + 71.5 - shift ) / 14.2 ) + exp ( - ( _lv + 89.0 - shift ) / 11.6 ) ) ) / tadj ;
   alpha = h_inf / tau_s ;
   beta = ( 1.0 - h_inf ) / tau_s ;
   k1ca = k2 * ( _lcai / cac ) * ( _lcai / cac ) * ( _lcai / cac ) * ( _lcai / cac ) ;
   k3p = k4 * ( p1 / Pc ) ;
    return 0; }
 
static void _hoc_evaluate_fct(void) {
  double _r;
   _r = 1.;
 evaluate_fct (  *getarg(1) , *getarg(2) );
 hoc_retpushx(_r);
}
 
static int  activation (  double _lv , double _lcai ) {
   double _lcc ;
 
/*VERBATIM*/
cai = _ion_cai;
 evaluate_fct ( _threadargscomma_ _lv , _lcai ) ;
   _lcc = 1.0 / ( 1.0 + pow( ( cac / _lcai ) , nca ) ) ;
   m = 1.0 / ( 1.0 + beta / alpha + pow( ( _lcc / Pc ) , nexp ) ) ;
   m = ( 1.0 + ginc * pow( ( _lcc / Pc ) , nexp ) ) * m ;
    return 0; }
 
static void _hoc_activation(void) {
  double _r;
   _r = 1.;
 activation (  *getarg(1) , *getarg(2) );
 hoc_retpushx(_r);
}
 
/*CVODE ode begin*/
 static int _ode_spec1() {_reset=0;{
 double b_flux, f_flux, _term; int _i;
 {int _i; for(_i=0;_i<5;_i++) _p[_dlist1[_i]] = 0.0;}
 evaluate_fct ( _threadargscomma_ v , cai ) ;
 /* ~ c1 <-> o1 ( alpha , beta )*/
 f_flux =  alpha * c1 ;
 b_flux =  beta * o1 ;
 Dc1 -= (f_flux - b_flux);
 Do1 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ p0 <-> p1 ( k1ca , k2 )*/
 f_flux =  k1ca * p0 ;
 b_flux =  k2 * p1 ;
 Dp0 -= (f_flux - b_flux);
 Dp1 += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ o1 <-> o2 ( k3p , k4 )*/
 f_flux =  k3p * o1 ;
 b_flux =  k4 * o2 ;
 Do1 -= (f_flux - b_flux);
 Do2 += (f_flux - b_flux);
 
 /*REACTION*/
   /* p0 + p1 = 1.0 */
 /*CONSERVATION*/
  /* c1 + o1 + o2 = 1.0 */
 /*CONSERVATION*/
   } return _reset;
 }
 
/*CVODE matsol*/
 static int _ode_matsol1() {_reset=0;{
 double b_flux, f_flux, _term; int _i;
   b_flux = f_flux = 0.;
 {int _i; double _dt1 = 1.0/dt;
for(_i=0;_i<5;_i++){
  	_RHS1(_i) = _dt1*(_p[_dlist1[_i]]);
	_MATELM1(_i, _i) = _dt1;
      
} }
 evaluate_fct ( _threadargscomma_ v , cai ) ;
 /* ~ c1 <-> o1 ( alpha , beta )*/
 _term =  alpha ;
 _MATELM1( 2 ,2)  += _term;
 _MATELM1( 3 ,2)  -= _term;
 _term =  beta ;
 _MATELM1( 2 ,3)  -= _term;
 _MATELM1( 3 ,3)  += _term;
 /*REACTION*/
  /* ~ p0 <-> p1 ( k1ca , k2 )*/
 _term =  k1ca ;
 _MATELM1( 4 ,4)  += _term;
 _MATELM1( 1 ,4)  -= _term;
 _term =  k2 ;
 _MATELM1( 4 ,1)  -= _term;
 _MATELM1( 1 ,1)  += _term;
 /*REACTION*/
  /* ~ o1 <-> o2 ( k3p , k4 )*/
 _term =  k3p ;
 _MATELM1( 3 ,3)  += _term;
 _MATELM1( 0 ,3)  -= _term;
 _term =  k4 ;
 _MATELM1( 3 ,0)  -= _term;
 _MATELM1( 0 ,0)  += _term;
 /*REACTION*/
   /* p0 + p1 = 1.0 */
 /*CONSERVATION*/
  /* c1 + o1 + o2 = 1.0 */
 /*CONSERVATION*/
   } return _reset;
 }
 
/*CVODE end*/
 
static int _ode_count(int _type){ return 5;}
 
static void _ode_spec(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  eh = _ion_eh;
  cai = _ion_cai;
     _ode_spec1 ();
  }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 5; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  eh = _ion_eh;
  cai = _ion_cai;
 _cvode_sparse(&_cvsparseobj1, 5, _dlist1, _p, _ode_matsol1, &_coef1);
 }}
 extern void nrn_update_ion_pointer(Symbol*, Datum*, int, int);
 static void _update_ion_pointer(Datum* _ppvar) {
   nrn_update_ion_pointer(_h_sym, _ppvar, 0, 0);
   nrn_update_ion_pointer(_h_sym, _ppvar, 1, 3);
   nrn_update_ion_pointer(_h_sym, _ppvar, 2, 4);
   nrn_update_ion_pointer(_ca_sym, _ppvar, 3, 1);
 }

static void initmodel() {
  int _i; double _save;_ninits++;
 _save = t;
 t = 0.0;
{
  c1 = c10;
  o2 = o20;
  o1 = o10;
  p1 = p10;
  p0 = p00;
 {
   tadj = pow( q10 , ( ( celsius - exptemp ) / 10.0 ) ) ;
   evaluate_fct ( _threadargscomma_ v , cai ) ;
   c1 = 1.0 ;
   o1 = 0.0 ;
   o2 = 0.0 ;
   p0 = 1.0 ;
   p1 = 0.0 ;
   }
  _sav_indep = t; t = _save;

}
}

static void nrn_init(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v = _v;
  eh = _ion_eh;
  cai = _ion_cai;
 initmodel();
 }}

static double _nrn_current(double _v){double _current=0.;v=_v;{ {
   m = o1 + ginc * o2 ;
   i = gmax * m * ( v - eh ) ;
   ih = i ;
   }
 _current += ih;

} return _current;
}

static void nrn_cur(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
  eh = _ion_eh;
  cai = _ion_cai;
 _g = _nrn_current(_v + .001);
 	{ double _dih;
  _dih = ih;
 _rhs = _nrn_current(_v);
  _ion_dihdv += (_dih - ih)/.001 ;
 	}
 _g = (_g - _rhs)/.001;
  _ion_ih += ih ;
#if CACHEVEC
  if (use_cachevec) {
	VEC_RHS(_ni[_iml]) -= _rhs;
  }else
#endif
  {
	NODERHS(_nd) -= _rhs;
  }
 
}}

static void nrn_jacob(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml];
#if CACHEVEC
  if (use_cachevec) {
	VEC_D(_ni[_iml]) += _g;
  }else
#endif
  {
     _nd = _ml->_nodelist[_iml];
	NODED(_nd) += _g;
  }
 
}}

static void nrn_state(_NrnThread* _nt, _Memb_list* _ml, int _type){
 double _break, _save;
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _nd = _ml->_nodelist[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 _break = t + .5*dt; _save = t;
 v=_v;
{
  eh = _ion_eh;
  cai = _ion_cai;
 { {
 for (; t < _break; t += dt) {
 error = sparse(&_sparseobj1, 5, _slist1, _dlist1, _p, &t, dt, ihkin,&_coef1, _linmat1);
 if(error){fprintf(stderr,"at line 147 in file Ih.mod:\n\n"); nrn_complain(_p); abort_run(error);}
 
}}
 t = _save;
 } }}

}

static void terminal(){}

static void _initlists() {
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = &(o2) - _p;  _dlist1[0] = &(Do2) - _p;
 _slist1[1] = &(p1) - _p;  _dlist1[1] = &(Dp1) - _p;
 _slist1[2] = &(c1) - _p;  _dlist1[2] = &(Dc1) - _p;
 _slist1[3] = &(o1) - _p;  _dlist1[3] = &(Do1) - _p;
 _slist1[4] = &(p0) - _p;  _dlist1[4] = &(Dp0) - _p;
_first = 0;
}
