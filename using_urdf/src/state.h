#include "hld.h"
#include <iostream>
namespace hld
{
class State
{
public:
  virtual void onEntry(hld *m) = 0;
  virtual void onDo(hld *m) = 0;
  virtual void onExit(hld *m) = 0;
};

class Ready : public State
{
private:
  std::string stateName;

public:
  Ready(hld *m);
  virtual ~Ready();
  void onEntry(hld *m);
  void onDo(hld *m);
  void onExit(hld *m);
};

class Initialized : public State
{
private:
public:
  Initialized(hld *m);
  virtual ~Initialized();
  void onEntry(hld *m);
  void onDo(hld *m);
  void onExit(hld *m);
};

class Active : public State
{
private:
public:
  Active(hld *m);
  virtual ~Active();
  void onEntry(hld *m);
  void onDo(hld *m);
  void onExit(hld *m);
};

class Emergency : public State
{
private:
  std::string stateName;

public:
  Emergency(hld *m);
  virtual ~Emergency();
  void onEntry(hld *m);
  void onDo(hld *m);
  void onExit(hld *m);
};

} // namespace hld