/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */



#include "optimization.h"

struct sConvert {
  ScalarFunction *sf;
  VectorFunction *vf;
  ConstrainedProblem *cp;
  KOrderMarkovFunction *kom;
  double(*cstyle_fs)(arr*, const arr&, void*);
  void (*cstyle_fv)(arr&, arr*, const arr&, void*);
  void *data;
  sConvert():sf(NULL),vf(NULL),cp(NULL),kom(NULL),cstyle_fs(NULL),cstyle_fv(NULL),data(NULL)/*,cs(NULL)*/ {};
  
  struct VectorFunction_ScalarFunction:ScalarFunction { //actual converter objects
    VectorFunction *f;
    VectorFunction_ScalarFunction(VectorFunction& _f):f(&_f) {}
    virtual double fs(arr& grad, arr& H, const arr& x);
  };

  struct KOrderMarkovFunction_VectorFunction:VectorFunction {
    KOrderMarkovFunction *f;
    KOrderMarkovFunction_VectorFunction(KOrderMarkovFunction& _f):f(&_f) {}
    virtual void fv(arr& y, arr& J, const arr& x);
  };

  struct KOrderMarkovFunction_ConstrainedProblem:ConstrainedProblem {
    KOrderMarkovFunction *f;
    KOrderMarkovFunction_ConstrainedProblem(KOrderMarkovFunction& _f):f(&_f) {}
    virtual double fc(arr& df, arr& Hf, arr& g, arr& Jg, const arr& x);
    virtual uint dim_x();
    virtual uint dim_g();
    uint dim_phi();
  };

  struct cfunc_ScalarFunction:ScalarFunction { //actual converter objects
    double (*f)(arr*, const arr&, void*);
    void *data;
    cfunc_ScalarFunction(double (*_f)(arr*, const arr&, void*),void *_data):f(_f), data(_data) {}
    virtual double fs(arr& grad, arr& H, const arr& x){  if(&H) NIY;    return f(&grad, x, data); }
  };

  struct cfunc_VectorFunction:VectorFunction { //actual converter objects
    void (*f)(arr&, arr*, const arr&, void*);
    void *data;
    cfunc_VectorFunction(void (*_f)(arr&, arr*, const arr&, void*),void *_data):f(_f), data(_data) {}
    virtual void fv(arr& y, arr& J, const arr& x){  f(y, &J, x, data);  }
  };
  
};

//the Convert is essentially only a ``garb_age collector'', creating all the necessary conversion objects and then deleting them on destruction
Convert::Convert(ScalarFunction& p) { s=new sConvert(); s->sf=&p; }
Convert::Convert(VectorFunction& p) { s=new sConvert(); s->vf=&p; }
//Convert::Convert(QuadraticFunction& p){ s=new sConvert(); s->sf=&p; }
//Convert::Convert(VectorChainFunction& p) { s=new sConvert(); s->vcf=&p; }
//Convert::Convert(QuadraticChainFunction& p) { s=new sConvert(); s->qcf=&p; }
Convert::Convert(KOrderMarkovFunction& p) { s=new sConvert(); s->kom=&p; }
Convert::Convert(double(*fs)(arr*, const arr&, void*),void *data) {  s=new sConvert(); s->cstyle_fs=fs; s->data=data; }
Convert::Convert(void (*fv)(arr&, arr*, const arr&, void*),void *data) {  s=new sConvert(); s->cstyle_fv=fv; s->data=data; }

#ifndef libRoboticsCourse
//Convert::Convert(ControlledSystem& p) { s=new sConvert(); s->cs=&p; }
#endif

Convert::~Convert() {
  delete s;
}

//===========================================================================
//
// casting methods
//

Convert::operator ScalarFunction&() {
  if(!s->sf) {
//    if(s->vcf) s->sf = new sConvert::VectorChainFunction_ScalarFunction(*s->vcf);
    if(s->kom) s->vf = new sConvert::KOrderMarkovFunction_VectorFunction(*s->kom);
    if(s->cstyle_fv) s->vf = new sConvert::cfunc_VectorFunction(s->cstyle_fv, s->data);
    if(s->vf)  s->sf = new sConvert::VectorFunction_ScalarFunction(*s->vf);
    if(s->cstyle_fs)  s->sf = new sConvert::cfunc_ScalarFunction(s->cstyle_fs, s->data);
  }
  if(!s->sf) HALT("");
  return *s->sf;
}

Convert::operator VectorFunction&() {
  if(!s->vf) {
//    if(s->cs) operator KOrderMarkovFunction&();
    if(s->kom) s->vf = new sConvert::KOrderMarkovFunction_VectorFunction(*s->kom);
//    if(s->vcf) s->vf = new sConvert::VectorChainFunction_VectorFunction(*s->vcf);
    if(s->cstyle_fv)  s->vf = new sConvert::cfunc_VectorFunction(s->cstyle_fv, s->data);
  }
  if(!s->vf) HALT("");
  return *s->vf;
}

Convert::operator ConstrainedProblem&() {
  if(!s->cp) {
    if(s->kom) s->cp = new sConvert::KOrderMarkovFunction_ConstrainedProblem(*s->kom);
  }
  if(!s->cp) HALT("");
  return *s->cp;
}

Convert::operator KOrderMarkovFunction&() {
  if(!s->kom) {
// #ifndef libRoboticsCourse
//     if(s->cs) s->kom = new sConvert::ControlledSystem_2OrderMarkovFunction(*s->cs);
// #endif
  }
  if(!s->kom) HALT("");
  return *s->kom;
}

//===========================================================================
//
// actual convertion routines
//

double sConvert::VectorFunction_ScalarFunction::fs(arr& grad, arr& H, const arr& x) {
  arr y,J;
  f->fv(y, (&grad?J:NoArr), x);
//  if(J.special==arr::RowShiftedPackedMatrixST) J = unpack(J);
  if(&grad){ grad = comp_At_x(J, y); grad *= 2.; }
  if(&H){ H = comp_At_A(J); H *= 2.; }
  return sumOfSqr(y);
}

void sConvert::KOrderMarkovFunction_VectorFunction::fv(arr& phi, arr& J, const arr& _x) {
#if 0 //non-packed Jacobian
  //probing dimensionality
  uint T=f->get_T();
  uint k=f->get_k();
  uint n=f->get_n();
  uint M=0;
  for(uint t=0; t<=T-k; t++) M+=f->get_m(t);
  CHECK(x.nd==2 && x.d1==n && x.d0==(T+1),"");
  //resizing things:
  phi.resize(M);   phi.setZero();
  if(&J) { J.resize(M,x.N); J.setZero(); }
  M=0;
  uint m_t;
  for(uint t=0; t<=T-k; t++) {
    m_t = f->get_m(t);
    arr phi_t,J_t;
    f->phi_t(phi_t, (&J?J_t:NoArr), t, x.subRange(t, t+k));
    CHECK(phi_t.N==m_t,"");
    phi.setVectorBlock(phi_t, M);
    if(&J) {
      if(J_t.nd==3) J_t.reshape(J_t.d0,J_t.d1*J_t.d2);
      CHECK(J_t.d0==m_t && J_t.d1==(k+1)*n,"");
      J.setMatrixBlock(J_t, M, t*n);
    }
    M += m_t;
  }
#else
  //probing dimensionality
  uint T=f->get_T();
  uint k=f->get_k();
  uint n=f->dim_x();
  uint dim_z=f->dim_z();
  uint dim_Phi=0;
  arr x_pre=f->get_prefix();
  arr x_post=f->get_postfix();
  arr x,z;
  if(dim_z){
    x.referTo(_x);
    x.reshape((T+1-x_post.d0)*n + dim_z);
    z.referToSubRange(x, -(int)dim_z, -1);
    x.referToSubRange(_x, 0, -(int)dim_z-1);
    x.reshape(T+1-x_post.d0, n);
  }else{
    x.referTo(_x);
    x.reshape(T+1-x_post.d0, n);
  }
  for(uint t=0; t<=T; t++) dim_Phi+=f->dim_phi(t);
  CHECK(x.nd==2 && x.d1==n && x.d0==T+1-x_post.d0,"");
  CHECK(x_pre.nd==2 && x_pre.d1==n && x_pre.d0==k,"prefix is of wrong dim");

  //resizing things:
  phi.resize(dim_Phi);   phi.setZero();
  RowShiftedPackedMatrix *Jaux, *Jzaux;
  arr *Jz;
  if(&J){
    Jaux = auxRowShifted(J, dim_Phi, (k+1)*n, _x.N);
    J.setZero();
    if(dim_z){
      Jz = new arr(dim_Phi, dim_z);
      Jz->setZero();
      Jaux->nextInSum = Jz;
      Jzaux = auxRowShifted(*Jz, dim_Phi, dim_z, _x.N);
    }
  }

  //loop over time t
  uint M=0;
  for(uint t=0; t<=T; t++) {
    uint dimf_t = f->dim_phi(t);
    if(!dimf_t) continue;

    //construct x_bar
    arr x_bar;
    if(t>=k) {
      if(t>=x.d0) { //x_bar includes the postfix
        x_bar.resize(k+1,n);
        for(int i=t-k; i<=(int)t; i++) x_bar[i-t+k]() = (i>=(int)x.d0)? x_post[i-x.d0] : x[i];
      } else{
        if(!dim_z) x_bar.referToSubRange(x, t-k, t);
        else x_bar = x.sub(t-k, t, 0, -1); //need to copy as we will augment
      }
    } else { //x_bar includes the prefix
      x_bar.resize(k+1,n);
      for(int i=t-k; i<=(int)t; i++) x_bar[i-t+k]() = (i<0)? x_pre[k+i] : x[i];
    }
    if(dim_z){ //append the constant variable to x_bar
      x_bar.insColumns(x_bar.d1, dim_z);
      for(uint i=0;i<=k;i++) x_bar[i].subRange(-dim_z, -1)=z;
    }

    //query
    arr f_t, J_t, Jz_t;
    f->phi_t(f_t, (&J?J_t:NoArr), t, x_bar);
    CHECK(f_t.N==dimf_t,"");
    phi.setVectorBlock(f_t, M);
    if(&J) {
      if(J_t.nd==3) J_t.reshape(J_t.d0,J_t.d1*J_t.d2);
      if(dim_z){//decompose J_t//TODO: inefficient
        Jz_t.resize(J_t.d0, dim_z).setZero();
        J_t.reshape(J_t.d0, (k+1)*(n+dim_z));
        for(uint i=0;i<=k;i++){
          J_t.setMatrixBlock(J_t.sub(0, -1, i*(n+dim_z), i*(n+dim_z)+n-1), 0, i*n);
          Jz_t += J_t.sub(0, -1, i*(n+dim_z)+n, i*(n+dim_z)+n+dim_z-1); //we add up the Jacobians
        }
        J_t.delColumns((k+1)*n, (k+1)*dim_z);
      }
      CHECK(J_t.d0==dimf_t && J_t.d1==(k+1)*n,"");
      if(t>=k) {
        J.setMatrixBlock(J_t, M, 0);
        for(uint i=0; i<dimf_t; i++) Jaux->rowShift(M+i) = (t-k)*n;
      } else { //cut away the Jacobian w.r.t. the prefix
        J_t.delColumns(0,(k-t)*n);
        J.setMatrixBlock(J_t, M, 0);
        for(uint i=0; i<dimf_t; i++) Jaux->rowShift(M+i) = 0;
      }
      if(dim_z){
        CHECK(!Jz_t.N || (Jz_t.d0==dimf_t && Jz_t.d1==dim_z),"");
        Jz->setMatrixBlock(Jz_t, M, 0);
        for(uint i=0; i<dimf_t; i++) Jzaux->rowShift(M+i) = x.N;
      }
    }
    M += dimf_t;
  }

  CHECK(M==dim_Phi,"");
  if(&J){
    Jaux->computeColPatches(true);
    if(dim_z) Jzaux->computeColPatches(false);
  }
#endif
}

//collect the constraints from a KOrderMarkovFunction
double sConvert::KOrderMarkovFunction_ConstrainedProblem::fc(arr& df, arr& Hf, arr& g, arr& Jg, const arr& x) {
#if 0 //old way
  //probing dimensionality
  uint T=f->get_T();
  uint k=f->get_k();
  uint n=f->dim_x();
  arr x_pre=f->get_prefix();
  arr x_post=f->get_postfix();

  CHECK(x.nd==2 && x.d1==n && x.d0==(T+1)-x_post.d0,"");
  CHECK(x_pre.nd==2 && x_pre.d1==n && x_pre.d0==k,"prefix is of wrong dim");
  CHECK(!x_post.N || (x_post.nd==2 && x_post.d1==n),"postfix is of wrong dim");

  //resizing things:
  uint meta_phid = dim_phi();
  uint meta_gd = dim_g();
  uint meta_yd = meta_phid - meta_gd;

  bool getJ = (&df || &Hf || &Jg);

  arr meta_y, meta_Jy;
  RowShiftedPackedMatrix *Jy_aux, *Jg_aux;
  meta_y.resize(meta_yd);
  if(&g) g.resize(meta_gd);
  if(getJ){ Jy_aux = auxRowShifted(meta_Jy, meta_yd, (k+1)*n, x.N); meta_Jy.setZero(); }
  if(&Jg){ Jg_aux = auxRowShifted(Jg, meta_gd, (k+1)*n, x.N); Jg.setZero(); }

  uint y_count=0;
  uint g_count=0;

  for(uint t=0; t<=T; t++) {
    uint phid = f->dim_phi(t);
    uint dimg_t   = f->dim_g(t);
    uint m_t   = phid-dimg_t;
    if(!phid) continue;
    arr x_bar, phi_t, J_t, f_t, Jf_t, g_t, Jg_t;

    //construct x_bar
    if(t>=k) {
      if(t>=x.d0) { //x_bar includes the postfix
        x_bar.resize(k+1,n);
        for(int i=t-k; i<=(int)t; i++) x_bar[i-t+k]() = (i>=(int)x.d0)? x_post[i-x.d0] : x[i];
      } else {
        x_bar.referToSubRange(x, t-k, t);
      }
    } else { //x_bar includes the prefix
      x_bar.resize(k+1,n);
      for(int i=t-k; i<=(int)t; i++) x_bar[i-t+k]() = (i<0)? x_pre[k+i] : x[i];
    }

    //query the phi
    f->phi_t(phi_t, (getJ?J_t:NoArr), t, x_bar);
    if(getJ) if(J_t.nd==3) J_t.reshape(J_t.d0, J_t.d1*J_t.d2);
    CHECK(phi_t.N==phid,"");
    if(getJ) CHECK(J_t.d0==phid && J_t.d1==(k+1)*n,"");

    //insert in meta_y
    f_t.referToSubRange(phi_t, 0, m_t-1);
    CHECK(f_t.N==m_t,"");
    meta_y.setVectorBlock(f_t, y_count);
    if(getJ) {
      Jf_t.referToSubRange(J_t, 0, m_t-1);
      if(t>=k) {
        meta_Jy.setMatrixBlock(Jf_t, y_count, 0);
        for(uint i=0; i<Jf_t.d0; i++) Jy_aux->rowShift(y_count+i) = (t-k)*n;
      } else { //cut away the Jacobian w.r.t. the prefix
        Jf_t.dereference();
        Jf_t.delColumns(0,(k-t)*n);
        meta_Jy.setMatrixBlock(Jf_t, y_count, 0);
        for(uint i=0; i<Jf_t.d0; i++) Jy_aux->rowShift(y_count+i) = 0;
      }
    }
    y_count += m_t;

    //insert in meta_g
    if(dimg_t){
      g_t.referToSubRange(phi_t, m_t, -1);
      CHECK(g_t.N==dimg_t,"");
      if(&g) g.setVectorBlock(g_t, g_count);
      if(&Jg) {
        Jg_t.referToSubRange(J_t, m_t, -1);
        if(t>=k) {
          Jg.setMatrixBlock(Jg_t, g_count, 0);
          for(uint i=0; i<Jg_t.d0; i++) Jg_aux->rowShift(g_count+i) = (t-k)*n;
        } else { //cut away the Jacobian w.r.t. the prefix
          Jg_t.dereference();
          Jg_t.delColumns(0,(k-t)*n);
          Jg.setMatrixBlock(Jg_t, g_count, 0);
          for(uint i=0; i<Jg_t.d0; i++) Jg_aux->rowShift(g_count+i) = 0;
        }
      }
      g_count += dimg_t;
    }
  }
  CHECK(y_count==meta_y.N,"");
  if(&g) CHECK(g_count==g.N,"");
  if(getJ) Jy_aux->computeColPatches(true);
  if(&Jg) Jg_aux->computeColPatches(true);
  //if(&J) J=Jaux->unpack();

  //finally, compute the scalar function
  if(&df){ df = comp_At_x(meta_Jy, meta_y); df *= 2.; }
  if(&Hf){ Hf = comp_At_A(meta_Jy); Hf *= 2.; }
  return sumOfSqr(meta_y);
#else
  sConvert::KOrderMarkovFunction_VectorFunction F(*f);
  arr phi, J;
  bool getJ = (&df) || (&Hf) || (&Jg);
  F.fv(phi, (getJ?J:NoArr), x);
  RowShiftedPackedMatrix *J_aux = (RowShiftedPackedMatrix*)J.aux;

  //resizing things:
  uint dimphi = dim_phi();
  uint dimg = dim_g();
  uint dimy = dimphi - dimg;
  CHECK(phi.N==dimphi,"");

  arr y, Jy;
  RowShiftedPackedMatrix *Jy_aux, *Jg_aux;
  y.resize(dimy);
  if(&g) g.resize(dimg);
  if(getJ) Jy_aux = auxRowShifted(Jy, dimy, J.d1, J_aux->real_d1);
  if(&Jg)  Jg_aux = auxRowShifted(Jg, dimg, J.d1, J_aux->real_d1);

  //if there is a z
  uint dimz = f->dim_z();
  arr *Jz, *Jzy, *Jzg;
  RowShiftedPackedMatrix *Jz_aux, *Jzy_aux, *Jzg_aux;
  if(dimz && getJ){
    Jz = J_aux->nextInSum;
    Jz_aux = (RowShiftedPackedMatrix*)Jz->aux;
            { Jzy = new arr(dimy, dimz);  Jy_aux->nextInSum = Jzy;  Jzy_aux = auxRowShifted(*Jzy, dimy, Jz->d1, Jz_aux->real_d1); }
    if(&Jg) { Jzg = new arr(dimg, dimz);  Jg_aux->nextInSum = Jzg;  Jzg_aux = auxRowShifted(*Jzg, dimg, Jz->d1, Jz_aux->real_d1); }
  }

  //loop over time t
  uint M=0, y_count=0, g_count=0;
  uint T=f->get_T();
  for(uint t=0; t<=T; t++){
    uint dimphi_t = f->dim_phi(t);
    uint dimg_t   = f->dim_g(t);
    uint dimf_t   = dimphi_t-dimg_t;

    //split up
    y.setVectorBlock(phi.subRange(M, M+dimf_t-1), y_count);
    if(getJ) {
      Jy.setMatrixBlock(J.subRange(M, M+dimf_t-1), y_count, 0);
      for(uint i=0; i<dimf_t; i++) Jy_aux->rowShift(y_count+i) = J_aux->rowShift(M+i);
      if(dimz){
        Jzy->setMatrixBlock(Jz->subRange(M, M+dimf_t-1), y_count, 0);
        for(uint i=0; i<dimf_t; i++) Jzy_aux->rowShift(y_count+i) = Jz_aux->rowShift(M+i);
      }
    }
    M += dimf_t;
    y_count += dimf_t;

    if(&g && dimg) g.setVectorBlock(phi.subRange(M, M+dimg_t-1), g_count);
    if(&Jg && dimg) {
      Jg.setMatrixBlock(J.subRange(M, M+dimg_t-1), g_count, 0);
      for(uint i=0; i<dimg_t; i++) Jg_aux->rowShift(g_count+i) = J_aux->rowShift(M+i);
      if(dimz){
        Jzg->setMatrixBlock(Jz->subRange(M, M+dimg_t-1), g_count, 0);
        for(uint i=0; i<dimg_t; i++) Jzg_aux->rowShift(g_count+i) = Jz_aux->rowShift(M+i);
      }
    }
    M += dimg_t;
    g_count += dimg_t;
  }
  CHECK(M==dimphi,"");
  CHECK(y_count==dimy,"");
  if(&g) CHECK(g_count==dimg,"");
  if(getJ) Jy_aux->computeColPatches(true);
  if(&Jg) Jg_aux->computeColPatches(true);

  //finally, compute the scalar function
  if(&df){ df = comp_At_x(Jy, y); df *= 2.; }
  if(&Hf){ Hf = comp_At_A(Jy); Hf *= 2.; }
  return sumOfSqr(y);
#endif
}

uint sConvert::KOrderMarkovFunction_ConstrainedProblem::dim_x() {
  uint T=f->get_T();
  uint n=f->dim_x();
  return (T+1)*n;
}

uint sConvert::KOrderMarkovFunction_ConstrainedProblem::dim_phi() {
  uint T =f->get_T();
  uint gphi=0;
  for(uint t=0; t<=T; t++) gphi += f->dim_phi(t);
  return gphi;
}

uint sConvert::KOrderMarkovFunction_ConstrainedProblem::dim_g() {
  uint T =f->get_T();
  uint gd=0;
  for(uint t=0; t<=T; t++) gd += f->dim_g(t);
  return gd;
}
