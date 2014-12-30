/*  ---------------------------------------------------------------------
    Copyright 2012 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de
    
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



// this requires T to be defined!!!

template struct MT::Array<T>;

template MT::Array<T> MT::operator~(const Array<T>& y);
template MT::Array<T> MT::operator-(const Array<T>& y, const Array<T>& z);
#ifndef NOFLOAT
template MT::Array<T> MT::operator*(const Array<T>& y, const Array<T>& z);
#endif
template MT::Array<T> MT::operator^(const Array<T>& y, const Array<T>& z);
template MT::Array<T> MT::operator%(const Array<T>& y, const Array<T>& z);
template MT::Array<T> MT::operator*(T, const Array<T>& z);
template MT::Array<T> MT::operator*(const Array<T>& z, T);
template MT::Array<T> MT::operator-(const Array<T>& z);
template MT::Array<T> MT::operator-(T, const Array<T>& z);
template MT::Array<T> MT::operator-(const Array<T>& z, T);
template MT::Array<T> MT::operator+(const Array<T>& y, const Array<T>& z);
template MT::Array<T> MT::operator+(T, const Array<T>& z);
template MT::Array<T>& MT::operator+=(Array<T>& y, T);
template MT::Array<T>& MT::operator+=(Array<T>& y, const Array<T>& z);
template MT::Array<T>& MT::operator-=(Array<T>& y, T);
template MT::Array<T>& MT::operator-=(Array<T>& y, const Array<T>& z);
template MT::Array<T>& MT::operator*=(Array<T>& y, T);
template MT::Array<T>& MT::operator/=(Array<T>& y, T);
template bool MT::operator==(const Array<T>& v, const Array<T>& w);
template bool MT::operator==(const Array<T>& v, const T *w);
template std::istream& MT::operator>>(std::istream& is, Array<T>& x);
template std::ostream& MT::operator<<(std::ostream& os, const Array<T>& x);

//BinaryOperation
template void transpose(MT::Array<T>& x, const MT::Array<T>& y);
template void inverse2d(MT::Array<T>& invA, const MT::Array<T>& A);

template T absMax(const MT::Array<T>& v);
template T absMin(const MT::Array<T>& v);
template T entropy(const MT::Array<T>& v);
template T normalizeDist(MT::Array<T>& v);
template void makeConditional(MT::Array<T>& P);
template void checkNormalization(MT::Array<T>& v, double tol);
template void eliminate(MT::Array<T>& x, const MT::Array<T>& y, uint d);
template void eliminate(MT::Array<T>& x, const MT::Array<T>& y, uint d, uint e);
template void eliminatePartial(MT::Array<T>& x, const MT::Array<T>& y, uint d);
template void checkNan(MT::Array<T> const&);

template T sqrDistance(const MT::Array<T>& v, const MT::Array<T>& w);
template T maxDiff(const MT::Array<T>& v, const MT::Array<T>& w, uint *im);
template T maxRelDiff(const MT::Array<T>& v, const MT::Array<T>& w, T tol);
//template T sqrDistance(const MT::Array<T>& v, const MT::Array<T>& w, const MT::Array<bool>& mask);
template T sqrDistance(const MT::Array<T>& g, const MT::Array<T>& v, const MT::Array<T>& w);
template T euclideanDistance(const MT::Array<T>& v, const MT::Array<T>& w);
template T metricDistance(const MT::Array<T>& g, const MT::Array<T>& v, const MT::Array<T>& w);

template T sum(const MT::Array<T>& v);
template T scalar(const MT::Array<T>& v);
template MT::Array<T> sum(const MT::Array<T>& v, uint d);
template T sumOfAbs(const MT::Array<T>& v);
template T sumOfSqr(const MT::Array<T>& v);
template T length(const MT::Array<T>& v);

template T var(const MT::Array<T>& v);
template T trace(const MT::Array<T>& v);

template MT::Array<T> log(const MT::Array<T>& v);
template MT::Array<T> exp(const MT::Array<T>& v);
template MT::Array<T> atan(const MT::Array<T>& v);
template MT::Array<T> pow(const MT::Array<T>& v,T);

template T minDiag(const MT::Array<T>& v);

template T product(const MT::Array<T>& v);
#ifndef NOFLOAT
template void innerProduct(MT::Array<T>& x, const MT::Array<T>& y, const MT::Array<T>& z);
#endif
template void outerProduct(MT::Array<T>& x, const MT::Array<T>& y, const MT::Array<T>& z);
template T scalarProduct(const MT::Array<T>& v, const MT::Array<T>& w);
template T scalarProduct(const MT::Array<T>& g, const MT::Array<T>& v, const MT::Array<T>& w);

template MT::Array<T> catCol(const MT::Array<MT::Array<T>* >& X);


template void tensorEquation(MT::Array<T> &X, const MT::Array<T> &A, const uintA &pickA, const MT::Array<T> &B, const uintA &pickB, uint sum);
template void tensorPermutation(MT::Array<T> &Y, const MT::Array<T> &X, const uintA &Yid);
template void tensorMarginal(MT::Array<T> &Y, const MT::Array<T> &X, const MT::Array<uint> &Yid);
template void tensorMaxMarginal(MT::Array<T> &Y, const MT::Array<T> &X, const MT::Array<uint> &Yid);
template void tensorMarginal_old(MT::Array<T> &y, const MT::Array<T> &x, const MT::Array<uint> &xd, const MT::Array<uint> &ids);
template void tensorMultiply(MT::Array<T> &X, const MT::Array<T> &Y, const MT::Array<uint> &Yid);
template void tensorMultiply_old(MT::Array<T> &x, const MT::Array<T> &y, const MT::Array<uint> &d, const MT::Array<uint> &ids);

template void rndInteger(MT::Array<T>& a, int low, int high, bool add);
template void rndUniform(MT::Array<T>& a, double low, double high, bool add);
template void rndGauss(MT::Array<T>& a, double stdDev, bool add);
//template void rndGauss(MT::Array<T>& a, bool add);
//template MT::Array<T>& rndGauss(double stdDev, uint dim);
template uint softMax(const MT::Array<T>& a, MT::Array<double>& soft, double beta);


#undef T
