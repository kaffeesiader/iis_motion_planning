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

#include "keyValueGraph.h"

//////////// taken from http://stackoverflow.com/questions/4532281/how-to-test-whether-class-b-is-derived-from-class-a
typedef char(&yes)[1];
typedef char(&no)[2];
template <typename B, typename D> struct Host {
  operator B*() const;
  operator D*();
};
template <typename B, typename D> struct is_base_of {
  template <typename T>
  static yes check(D*, T);
  static no check(B*, int);
  static const bool value = sizeof(check(Host<B,D>(), int())) == sizeof(yes);
};
///////////////STOP

//===========================================================================
//
//  typed Item
//

template<class T>
struct Item_typed:Item {
  T *value;
  
  Item_typed():value(NULL) {}
  Item_typed(T *_value):value(_value) {}
  //copy value
  Item_typed(const StringA& _keys, const ItemL& _parents, const T& _value, KeyValueGraph *container=NULL):value(NULL) {
    value = new T(_value);
    keys=_keys;
    parents=_parents;
    if(container) container->append(this);
  }
  
  //directly store pointer to value
  Item_typed(const StringA& _keys, const ItemL& _parents, T *_value=NULL, KeyValueGraph *container=NULL):value(_value) {
    keys=_keys;
    parents=_parents;
    if(container) container->append(this);
  }

  virtual bool hasValue() const {
    return value!=NULL;
  }

  virtual void takeoverValue(Item *it) {
    Item_typed<T> *itt = dynamic_cast<Item_typed<T>*>(it);
    CHECK(itt,"can't assign to wrong type");
    CHECK(itt->value,"can't assign to nothing");
    if(value) delete value;
    value = itt->value;
    itt->value = NULL;
  }

  virtual void writeValue(std::ostream &os) const {
    if(typeid(T)==typeid(ItemL)) listWrite(*(ItemL*)(value), os, " ");
    else os <<*value;
  }
  
  virtual const std::type_info& getValueType() const {
    return typeid(T);
  }
  
  virtual bool is_derived_from_RootType() const {
    return is_base_of<RootType, T>::value;
  }
  
  virtual Item *newClone() const { return new Item_typed<T>(keys, parents, value); }
};

template<class T> T *Item::getValue() {
  Item_typed<T>* typed = dynamic_cast<Item_typed<T>*>(this);
  if(!typed) {
    if(getValueType() == typeid(KeyValueGraph)){ //try to get the item from the key value graph
      const KeyValueGraph *kvg = getValue<KeyValueGraph>();
      if(kvg->N==1){ //only if it has size 1??
        typed = dynamic_cast<Item_typed<T>*>(kvg->elem(0));
      }
    }
    if(!typed){
      MT_MSG("can't cast type '" <<getValueType().name() <<"' to type '" <<typeid(T).name() <<"' -- returning NULL");
      return NULL;
    }
  }
  return typed->value;
}

template<class T> const T *Item::getValue() const {
  const Item_typed<T>* typed = dynamic_cast<const Item_typed<T>*>(this);
  if(!typed) {
    if(getValueType() == typeid(KeyValueGraph)){ //try to get the item from the key value graph
      const KeyValueGraph *kvg = getValue<KeyValueGraph>();
      if(kvg->N==1){ //only if it has size 1??
        typed = dynamic_cast<const Item_typed<T>*>(kvg->elem(0));
      }
    }
    MT_MSG("can't cast type '" <<getValueType().name() <<"' to type '" <<typeid(T).name() <<"' -- returning reference-to-NULL");
    return NULL;
  }
  return typed->value;
}

template<class T> T* KeyValueGraph::getValue(const char *key) {
  Item *it = getItem(key);
  if(!it) return NULL;
  return it->getValue<T>();
}

template<class T> MT::Array<T*> KeyValueGraph::getTypedValues(const char* key) {
  MT::Array<T*> ret;
  for(Item *it: (*this)) if(it->getValueType()==typeid(T)) {
    if(!key) ret.append(it->getValue<T>());
    else for(uint i=0; i<it->keys.N; i++) if(it->keys(i)==key) {
      ret.append(it->getValue<T>());
      break;
    }
  }
  return ret;
}

template<class T> Item *KeyValueGraph::append(const StringA& keys, const ItemL& parents, T *x) {
  Item *it = append(new Item_typed<T>(keys, parents, x, NULL));

  for(Item *par: parents) par->parentOf.append(it);
  return it;
}

template <class T> MT::Array<T*> KeyValueGraph::getDerivedValues() {
  MT::Array<T*> ret;
  for(Item *it: (*this)) {
    if(it->is_derived_from_RootType()) {
      T *val= dynamic_cast<T*>(((Item_typed<RootType>*)it)->value);
      if(val) ret.append(val);
    }
  }
  return ret;
}

template <class T> ItemL KeyValueGraph::getDerivedItems() {
  ItemL ret;
  for(Item *it: (*this)) {
    if(it->is_derived_from_RootType()) {
      T *val= dynamic_cast<T*>(((Item_typed<RootType>*)it)->value);
      if(val) ret.append(it);
    }
  }
  return ret;
}
