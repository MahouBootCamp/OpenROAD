#ifndef UV_DRC_HH
#define UV_DRC_HH

#include <unordered_map>
#include <cstddef>
#include <memory>

#include <stt/SteinerTreeBuilder.h>
#include <rsz/Resizer.hh>
#include <sta/Delay.hh>
#include <sta/Network.hh>
#include <sta/Path.hh>
#include "odb/geom.h"

namespace uv_drc {

enum class RCTreeNodeType {
  LOAD,
  WIRE,
  JUNCTION
};

class LocEqual {
 public:
  bool operator()(const odb::Point& loc1, const odb::Point& loc2) const;
};

class LocHash {
 public:
  size_t operator()(const odb::Point& loc) const;

 private:
  std::size_t HashCombine(std::size_t seed, std::size_t value) const;
};

class RCTreeNode {
public:
  virtual ~RCTreeNode() = default;
  virtual RCTreeNodeType type() const = 0;
  odb::Point location() const { return loc_; }
private:
  odb::Point loc_;
};

using RCTreeNodePtr = std::shared_ptr<RCTreeNode>;

class LoadNode: public RCTreeNode {
public:
  ~LoadNode() override = default;
  RCTreeNodeType type() const override { return RCTreeNodeType::LOAD; }
private:
  sta::Pin* pin_;
};

class WireNode: RCTreeNode {
public:
  ~WireNode() override = default;
  RCTreeNodeType type() const override { return RCTreeNodeType::WIRE; }
private:
  RCTreeNodePtr downstream_;
};

class JuncNode: RCTreeNode {
public:
  ~JuncNode() override = default;
  RCTreeNodeType type() const override { return RCTreeNodeType::JUNCTION; }
private:
  RCTreeNodePtr child1_;
  RCTreeNodePtr child2_;
};

using LocMap
    = std::unordered_map<odb::Point, const sta::Pin*, LocHash, LocEqual>;
using LocVec = std::vector<odb::Point>;

// class RCTree {
// public:
//   RCTree();
// private:
//   RCTreeNodePtr root_ = nullptr;
// };

// UvDRCSlewBuffer targets to fix slew violations by inserting buffers.
class UvDRCSlewBuffer
{
 public:
  UvDRCSlewBuffer(rsz::Resizer* resizer,
                  const sta::Pin* pin,
                  const sta::Corner* corner,
                  int max_cap)
      : resizer_(resizer),
        drvr_pin_(pin),
        corner_(corner),
        max_cap_(max_cap)
  {
  }
  ~UvDRCSlewBuffer() = default;
  void TestFunction();

 private:
  std::tuple<LocVec, LocMap> InitNetConnections();
  stt::Tree makeSteinerTree(LocVec& locs, LocMap& loc_map);

 private:
  rsz::Resizer* resizer_;
  const sta::Pin* drvr_pin_;
  const sta::Corner* corner_;
  int max_cap_;
};

}  // namespace uv_drc

#endif
