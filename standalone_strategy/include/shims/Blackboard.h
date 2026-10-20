#pragma once

#include <memory>
#include <unordered_map>
#include <string>
#include <typeindex>
#include <typeinfo>
#include <type_traits>
#include <cassert>

class Streamable;
class In;

struct HasReadWrite
{
  template<typename T> static auto test(T* t) -> decltype(t->read(*static_cast<In*>(nullptr)), bool()) { return true; }
  static bool test(void*) { return false; }
};

class Blackboard
{
private:
  struct Entry
  {
    std::shared_ptr<void> data;
    Streamable* streamable = nullptr;
    int counter = 0;
  };

  std::unordered_map<std::string, Entry> entriesByName;
  std::unordered_map<std::type_index, Entry*> entriesByType;
  int version = 0;

public:
  static Blackboard& getInstance()
  {
    static Blackboard instance;
    return instance;
  }

  Blackboard() = default;
  ~Blackboard() = default;

  bool exists(const char* representation) const
  {
    auto it = entriesByName.find(representation);
    return it != entriesByName.end() && it->second.data != nullptr;
  }

  template<typename T>
  T& alloc(const char* representation)
  {
    Entry& entry = entriesByName[representation];
    entry.counter++;
    if(!entry.data)
    {
      auto p = std::make_shared<T>();
      entry.data = p;
      if constexpr (std::is_base_of_v<Streamable, T>)
        entry.streamable = static_cast<Streamable*>(p.get());
      entriesByType[std::type_index(typeid(T))] = &entry;
      ++version;
    }
    return *static_cast<T*>(entry.data.get());
  }

  template<typename T>
  T& get()
  {
    auto it = entriesByType.find(std::type_index(typeid(T)));
    assert(it != entriesByType.end() && it->second->data != nullptr);
    return *static_cast<T*>(it->second->data.get());
  }

  template<typename T>
  const T& get() const
  {
    auto it = entriesByType.find(std::type_index(typeid(T)));
    assert(it != entriesByType.end() && it->second->data != nullptr);
    return *static_cast<const T*>(it->second->data.get());
  }

  void free(const char* representation)
  {
    auto it = entriesByName.find(representation);
    if(it != entriesByName.end())
      it->second.counter--;
  }

  template<typename T>
  void free(const char* representation = nullptr)
  {
    if(representation)
      free(representation);
  }

  void reset(const char*) {}

  Streamable& operator[](const char* representation)
  {
    auto it = entriesByName.find(representation);
    assert(it != entriesByName.end() && it->second.streamable != nullptr);
    return *it->second.streamable;
  }

  const Streamable& operator[](const char* representation) const
  {
    auto it = entriesByName.find(representation);
    assert(it != entriesByName.end() && it->second.streamable != nullptr);
    return *it->second.streamable;
  }

  int getVersion() const { return version; }
};
