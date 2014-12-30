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


/// @file
/// @ingroup group_Core
/// @addtogroup group_Core
/// @{

#ifndef MT_registry_h
#define MT_registry_h

#include "array.h"
#include "keyValueGraph.h"


//===========================================================================
//
// global registry of anything using a singleton KeyValueGraph
//

KeyValueGraph& registry();

//macros to be used in *.cpp files

#define REGISTER_ITEM(T, key, value) \
  Item_typed<T > key##_RegistryEntry(ARRAY<MT::String>(MT::String(#key)), ItemL(), value, &registry());

#define REGISTER_ITEM2(T, key1, key2, value) \
  Item_typed<T > key1##_##key2##_RegistryEntry(ARRAY<MT::String>(MT::String(#key1),MT::String(#key2)), ItemL(), value, &registry());


//===========================================================================
//
// to register a type (instead of general thing/item), use this:
//

struct Type:RootType {
  MT::Array<Type*> parents; //TODO -> remove; replace functionality from registry
  virtual const std::type_info& typeId() const {NIY}; //TODO -> typeid()
  virtual struct Item* readItem(istream&) const {NIY}; //TODO -> readIntoNewItem
  virtual void* newInstance() const {NIY}
  virtual Type* clone() const {NIY}
  void write(std::ostream& os) const {
    os <<"Type '" <<typeId().name() <<"' ";
    if(parents.N) {
      cout <<"parents=[";
      for(Type *p: parents) cout <<' ' <<p->typeId().name();
      cout <<" ]";
    }
  }
  void read(std::istream& is) const { NIY; }
};
stdPipes(Type);

inline bool operator!=(Type& t1, Type& t2){ return t1.typeId()!= t2.typeId(); }

typedef MT::Array<Type*> TypeInfoL;


//===========================================================================
//
// retrieving types
//

//-- query existing types
inline Item *reg_findType(const char* key) {
  ItemL types = registry().getDerivedItems<Type>();
  for(Item *ti: types) {
    if(MT::String(ti->getValue<Type>()->typeId().name())==key) return ti;
    for(uint i=0; i<ti->keys.N; i++) if(ti->keys(i)==key) return ti;
  }
  return NULL;
}

template<class T>
Item *reg_findType() {
  ItemL types = registry().getDerivedItems<Type>();
  for(Item *ti: types) {
    if(ti->getValue<Type>()->typeId()==typeid(T)) return ti;
  }
  return NULL;
}


//===========================================================================
//
// read a value from a stream by looking up available registered types
//

inline Item* readTypeIntoItem(const char* key, std::istream& is) {
  TypeInfoL types = registry().getDerivedValues<Type>();
  Item *ti = reg_findType(key);
  if(ti) return ti->getValue<Type>()->readItem(is);
  return NULL;
}


//===========================================================================
//
// typed version
//

template<class T, class Base>
struct Type_typed:Type {
  Type_typed() {}
  Type_typed(const char *userBase, TypeInfoL *container) {
    if(userBase) {
      Item *it=reg_findType<Base>();
      if(it) parents.append(it->getValue<Type>());
    }
    if(container) {
      container->append(this);
    }
  }
  virtual const std::type_info& typeId() const { return typeid(T); }
  virtual Item* readItem(istream& is) const { T *x=new T(); is >>*x; return new Item_typed<T>(x); }
  virtual void* newInstance() const { return new T(); }
  virtual Type* clone() const { Type *t = new Type_typed<T, void>(); t->parents=parents; return t; }
};


//===========================================================================
//
// macros for declaring types (in *.cpp files)
//

#define KO ,
#define REGISTER_TYPE(T) \
  REGISTER_ITEM2(Type, Decl_Type, T, new Type_typed<T KO void>(NULL,NULL));

#define REGISTER_TYPE_Key(Key, T) \
  REGISTER_ITEM2(Type, Decl_Type, Key, new Type_typed<T KO void>(NULL,NULL));

#define REGISTER_TYPE_DERIVED(T, Base) \
  REGISTER_ITEM2(Type, Decl_Type, T, new Type_typed<T KO Base>(#Base,NULL));


#endif

/// @} //end group
