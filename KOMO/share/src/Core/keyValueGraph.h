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

#ifndef MT_keyValueGraph_h
#define MT_keyValueGraph_h

#include "array.h"

struct Item;
struct KeyValueGraph;
typedef MT::Array<Item*> ItemL;
typedef MT::Array<MT::String> StringA;
extern const ItemL& NoItemL;
struct RootType { virtual ~RootType() {}; }; ///< if types derive from RootType, more tricks are possible
inline std::istream& operator>>(std::istream&, RootType&) { NIY; }
inline std::ostream& operator<<(std::ostream&, const RootType&) { NIY; }

struct Item {
  StringA keys;
  ItemL parents;
  ItemL parentOf;
  uint index;
  virtual ~Item() {};
  template<class T> T *getValue();    ///< query whether the Item is of a certain value, return the value if so
  template<class T> const T *getValue() const; ///< as above
  void write(std::ostream &os) const;

  //-- virtuals implemented by Item_typed
  virtual bool hasValue() const {NIY};
  virtual void writeValue(std::ostream &os) const {NIY}
  virtual const std::type_info& getValueType() const {NIY}
  virtual bool is_derived_from_RootType() const {NIY}
  virtual void takeoverValue(Item*) {NIY}
  virtual Item *newClone() const {NIY}
};
stdOutPipe(Item);


struct KeyValueGraph:ItemL {
  struct sKeyValueGraph *s;
  KeyValueGraph *parentKvg;
  bool isReference;
  
  KeyValueGraph();
  ~KeyValueGraph();
  
  KeyValueGraph& operator=(const KeyValueGraph&);
  ItemL& list() { return *this; }
  
  //-- get values directly
  template<class T> T* getValue(const char *key);
  template<class T> bool getValue(T& x, const char *key) { T* y=getValue<T>(key); if(y) { x=*y; return true; } return false; }

  //-- get items
  Item* getItem(const char *key);
  Item* getItem(const char *key1, const char *key2);
  Item* operator[](const char *key) { return getItem(key); }
  
  //-- get lists of items
  KeyValueGraph getItems(const char* key);
  KeyValueGraph getTypedItems(const char* key, const std::type_info& type);
  template<class T> KeyValueGraph getTypedItems(const char* key){ return getTypedItems(key, typeid(T)); }
  template<class T> ItemL getDerivedItems();

  //-- get lists of values
  template<class T> MT::Array<T*> getTypedValues(const char* key);
  template<class T> MT::Array<T*> getDerivedValues();
  
  //-- adding items
  Item *append(Item* it) { it->index=ItemL::N;  ItemL::append(it);  return it;}
  template<class T> Item *append(const StringA& keys, const ItemL& parents, T *x);
  template<class T> Item *append(const StringA& keys, T *x) { return append(keys, ItemL(), x); }
  template<class T> Item *append(const char *key, T *x) { return append(ARRAY<MT::String>(MT::String(key)), ItemL(), x); }
  template<class T> Item *append(const char *key1, const char* key2, T *x) {  return append(ARRAY<MT::String>(MT::String(key1), MT::String(key2)), ItemL(), x); }

  //-- merging items
  Item *merge(Item* m); //removes m and deletes, if it is a member of This and merged with another Item
  void merge(const ItemL& L){ for(Item *m:L) merge(m); }

  //-- I/O
  void sortByDotOrder();
  
  void read(std::istream& is);
  void write(std::ostream& os=std::cout, const char *ELEMSEP="\n", const char *delim=NULL) const;
  void writeDot(const char* filename="z.dot");
};
stdPipes(KeyValueGraph);

#include "keyValueGraph_t.h"

#endif

/// @} //end group
