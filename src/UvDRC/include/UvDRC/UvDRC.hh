#ifndef UV_DRC_HH
#define UV_DRC_HH

#include <stt/SteinerTreeBuilder.h>

#include <cstddef>
#include <memory>
#include <rsz/Resizer.hh>
#include <sta/Delay.hh>
#include <sta/Network.hh>
#include <sta/Path.hh>
#include <stdexcept>
#include <unordered_map>

#include "db_sta/dbSta.hh"
#include "odb/geom.h"

namespace uv_drc {

class LocEqual
{
 public:
  bool operator()(const odb::Point& loc1, const odb::Point& loc2) const;
};

class LocHash
{
 public:
  size_t operator()(const odb::Point& loc) const;

 private:
  std::size_t HashCombine(std::size_t seed, std::size_t value) const;
};

class RCTreeNode;
using RCTreeNodePtr = std::shared_ptr<RCTreeNode>;
enum class RCTreeNodeType
{
  LOAD,
  WIRE,
  JUNCTION,
  DRIVER
};

class RCTreeNode
{
 public:
  RCTreeNode(odb::Point loc) : loc_(loc) {}
  virtual ~RCTreeNode() = default;
  virtual RCTreeNodeType Type() const = 0;
  odb::Point Location() const { return loc_; }
  virtual void AddDownstreamNode(RCTreeNodePtr ptr) = 0;

 private:
  odb::Point loc_;
};

class LoadNode : public RCTreeNode
{
 public:
  LoadNode(odb::Point loc, const sta::Pin* pin) : RCTreeNode(loc), pin_(pin) {}
  ~LoadNode() override = default;
  RCTreeNodeType Type() const override { return RCTreeNodeType::LOAD; }
  void AddDownstreamNode(RCTreeNodePtr ptr) override
  {
    throw std::runtime_error("LoadNode cannot have downstream nodes.");
  }

 private:
  const sta::Pin* pin_;
};

class DrivNode : public RCTreeNode
{
 public:
  DrivNode(odb::Point loc, const sta::Pin* pin) : RCTreeNode(loc), pin_(pin) {}
  ~DrivNode() override = default;
  RCTreeNodeType Type() const override { return RCTreeNodeType::DRIVER; }
  void AddDownstreamNode(RCTreeNodePtr ptr) override
  {
    if (downstream_ != nullptr) {
      throw std::runtime_error("Driver node can only have one downstream node.");
    }
    downstream_ = ptr;
  }

 private:
  const sta::Pin* pin_;
  RCTreeNodePtr downstream_;
};

class WireNode : public RCTreeNode
{
 public:
  WireNode(odb::Point loc) : RCTreeNode(loc) {}
  ~WireNode() override = default;
  RCTreeNodeType Type() const override { return RCTreeNodeType::WIRE; }
  void AddDownstreamNode(RCTreeNodePtr ptr) override
  {
    if (downstream_ != nullptr) {
      throw std::runtime_error("Wire node can only have one downstream node.");
    }
    downstream_ = ptr;
  }

 private:
  RCTreeNodePtr downstream_;
};

class JuncNode : public RCTreeNode
{
 public:
  JuncNode(odb::Point loc) : RCTreeNode(loc) {}
  ~JuncNode() override = default;
  RCTreeNodeType Type() const override { return RCTreeNodeType::JUNCTION; }
  void AddDownstreamNode(RCTreeNodePtr ptr) override
  {
    children_.push_back(ptr);
  }

 private:
  std::vector<RCTreeNodePtr> children_;
};

using LocMap
    = std::unordered_map<odb::Point, const sta::Pin*, LocHash, LocEqual>;
using LocVec = std::vector<odb::Point>;

// UvDRCSlewBuffer targets to fix slew violations by inserting buffers.
class UvDRCSlewBuffer
{
 public:
  UvDRCSlewBuffer(rsz::Resizer* resizer,
                  const sta::Pin* pin,
                  const sta::Corner* corner,
                  int max_cap)
      : resizer_(resizer), drvr_pin_(pin), corner_(corner), max_cap_(max_cap)
  {
  }
  ~UvDRCSlewBuffer() = default;
  void TestFunction();
  void Run();

 private:
  std::tuple<LocVec, LocMap> InitNetConnections();
  stt::Tree MakeSteinerTree(LocVec& locs, LocMap& loc_map);
  void BuildRCTree(stt::Tree& tree, LocVec& locs, LocMap& loc_map);
  void PrepareBufferSlots();

 private:
  rsz::Resizer* resizer_;
  const sta::Pin* drvr_pin_;
  const sta::Corner* corner_;
  int max_cap_;
  RCTreeNodePtr root_ = nullptr;
};

}  // namespace uv_drc

#endif
