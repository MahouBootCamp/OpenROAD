#ifndef UV_DRC_HH
#define UV_DRC_HH

#include <odb/geom.h>
#include <stt/SteinerTreeBuilder.h>

#include <cstddef>
#include <db_sta/dbSta.hh>
#include <memory>
#include <rsz/Resizer.hh>
#include <sta/Delay.hh>
#include <sta/Network.hh>
#include <sta/Path.hh>
#include <stdexcept>

#include "sta/Corner.hh"
#include "utl/Logger.h"

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

using LocVec = std::vector<odb::Point>;
using PinVec = std::vector<const sta::Pin*>;

enum class UvDRCSlewModel
{
  Alpert,
  OpenROAD
};

class RCTreeNode;
using RCTreeNodePtr = std::shared_ptr<RCTreeNode>;
enum class RCTreeNodeType
{
  LOAD,
  WIRE,
  JUNCTION,
  DRIVER,
  NONE
};

class RCTreeNode
{
 public:
  RCTreeNode(odb::Point loc, RCTreeNodeType type) : loc_(loc), type_(type) {}
  virtual ~RCTreeNode() = default;
  RCTreeNodeType Type() { return type_; }
  odb::Point Location() const { return loc_; }
  virtual void AddDownstreamNode(RCTreeNodePtr ptr) = 0;
  virtual void RemoveDownstreamNode(RCTreeNodePtr ptr) = 0;
  virtual std::vector<RCTreeNodePtr> DownstreamNodes() = 0;
  virtual std::size_t DownstreamNodeCount() = 0;

  void DebugPrint(int indent, utl::Logger* logger);

 private:
  odb::Point loc_;
  RCTreeNodeType type_;
};

class LoadNode : public RCTreeNode
{
 public:
  LoadNode(odb::Point loc, const sta::Pin* pin)
      : RCTreeNode(loc, RCTreeNodeType::LOAD), pin_(pin)
  {
  }
  ~LoadNode() override = default;
  void AddDownstreamNode(RCTreeNodePtr ptr) override
  {
    throw std::runtime_error("LoadNode cannot have downstream nodes.");
  }
  void RemoveDownstreamNode(RCTreeNodePtr ptr) override
  {
    throw std::runtime_error("LoadNode cannot have downstream nodes.");
  }
  std::vector<RCTreeNodePtr> DownstreamNodes() override { return {}; }
  std::size_t DownstreamNodeCount() override { return 0; }

 private:
  const sta::Pin* pin_;
};

class DrivNode : public RCTreeNode
{
 public:
  DrivNode(odb::Point loc, const sta::Pin* pin)
      : RCTreeNode(loc, RCTreeNodeType::DRIVER), pin_(pin)
  {
  }
  ~DrivNode() override = default;
  void AddDownstreamNode(RCTreeNodePtr ptr) override
  {
    if (downstream_ != nullptr) {
      throw std::runtime_error(
          "Driver node can only have one downstream node.");
    }
    downstream_ = ptr;
  }
  std::vector<RCTreeNodePtr> DownstreamNodes() override
  {
    return {downstream_};
  }
  void RemoveDownstreamNode(RCTreeNodePtr ptr) override
  {
    if (downstream_ != ptr) {
      throw std::runtime_error(
          "Remove non-matching downstream node from Driver node.");
    }
    downstream_ = nullptr;
  }
  std::size_t DownstreamNodeCount() override
  {
    return downstream_ == nullptr ? 0 : 1;
  }

 private:
  const sta::Pin* pin_;
  RCTreeNodePtr downstream_;
};

class WireNode : public RCTreeNode
{
 public:
  WireNode(odb::Point loc) : RCTreeNode(loc, RCTreeNodeType::WIRE) {}
  ~WireNode() override = default;
  void AddDownstreamNode(RCTreeNodePtr ptr) override
  {
    if (downstream_ != nullptr) {
      throw std::runtime_error("Wire node can only have one downstream node.");
    }
    downstream_ = ptr;
  }
  std::vector<RCTreeNodePtr> DownstreamNodes() override
  {
    return {downstream_};
  }
  void RemoveDownstreamNode(RCTreeNodePtr ptr) override
  {
    if (downstream_ != ptr) {
      throw std::runtime_error(
          "Remove non-matching downstream node from Wire node.");
    }
    downstream_ = nullptr;
  }
  std::size_t DownstreamNodeCount() override
  {
    return downstream_ == nullptr ? 0 : 1;
  }

 private:
  RCTreeNodePtr downstream_;
};

class JuncNode : public RCTreeNode
{
 public:
  JuncNode(odb::Point loc) : RCTreeNode(loc, RCTreeNodeType::JUNCTION) {}
  ~JuncNode() override = default;
  void AddDownstreamNode(RCTreeNodePtr ptr) override
  {
    if (downstream1_ == nullptr) {
      downstream1_ = ptr;
    } else if (downstream2_ == nullptr) {
      downstream2_ = ptr;
    } else {
      throw std::runtime_error("Junc node can only have two downstream nodes.");
    }
  }
  std::vector<RCTreeNodePtr> DownstreamNodes() override
  {
    return {downstream1_, downstream2_};
  }
  void RemoveDownstreamNode(RCTreeNodePtr ptr) override
  {
    if (downstream1_ == ptr) {
      downstream1_ = nullptr;
    } else if (downstream2_ == ptr) {
      downstream2_ = nullptr;
    } else {
      throw std::runtime_error(
          "Remove non-matching downstream node from Junc node.");
    }
  }
  std::size_t DownstreamNodeCount() override
  {
    std::size_t count = 0;
    if (downstream1_ != nullptr) {
      count++;
    }
    if (downstream2_ != nullptr) {
      count++;
    }
    return count;
  }

 private:
  RCTreeNodePtr downstream1_;
  RCTreeNodePtr downstream2_;
};

// UvDRCSlewBuffer targets to fix slew violations by inserting buffers.
class UvDRCSlewBuffer
{
 public:
  UvDRCSlewBuffer(rsz::Resizer* resizer,
                  UvDRCSlewModel model = UvDRCSlewModel::OpenROAD)
      : resizer_(resizer), model_(model)
  {
  }
  ~UvDRCSlewBuffer() = default;
  void Run(const sta::Pin* drvr_pin, const sta::Corner* corner, int max_cap);

 private:
  struct BufferCandidate
  {
    sta::LibertyCell* cell;
    float input_cap;
    float drive_resistance;
  };

 private:
  void TestFunction();
  void InitBufferCandidates();
  void SetMaxWireLength(int max_length_dbu)
  {
    user_max_wire_length_ = max_length_dbu;
  }

  std::tuple<LocVec, PinVec> InitNetConnections(const sta::Pin* drvr_pin);
  stt::Tree MakeSteinerTree(const sta::Pin* drvr_pin,
                            LocVec& locs,
                            PinVec& pins);
  RCTreeNodePtr BuildRCTree(const sta::Pin* drvr_pin,
                            stt::Tree& tree,
                            LocVec& locs,
                            PinVec& pins);
  RCTreeNodePtr BuildRCTreeHelper(
      std::vector<RCTreeNodePtr>& nodes,
      std::vector<std::vector<std::size_t>>& adjacents,
      std::size_t current_index);
  void PrepareBufferSlots(RCTreeNodePtr root, const sta::Corner* corner);
  void PrepareBufferSlotsHelper(RCTreeNodePtr u,
                                RCTreeNodePtr d,
                                int buffer_step);

  int MaxLengthForSlew(sta::LibertyCell* buffer_cell,
                       const sta::Corner* corner);
  int MaxLengthForSlewAlpert(sta::LibertyCell* buffer_cell,
                             const sta::Corner* corner);
  int MaxLengthForSlewOpenROAD(sta::LibertyCell* buffer_cell,
                               const sta::Corner* corner);
  int MaxLengthForCap(sta::LibertyCell* buffer_cell, const sta::Corner* corner);

 private:
  rsz::Resizer* resizer_;
  // const sta::Pin* drvr_pin_;
  // const sta::Corner* corner_;
  // int max_cap_;
  UvDRCSlewModel model_;

  std::vector<BufferCandidate> buffer_candidates_;
  // RCTreeNodePtr root_ = nullptr;
  // TODO: DELETE THIS
  int user_max_wire_length_ = std::numeric_limits<int>::max();  // in DBU

  static constexpr float k_slew_margin_ = 0.2f;  // 20%
  static constexpr float k_cap_margin_ = 0.2f;   // 20%
  static constexpr float k_openroad_slew_factor = 1.39;
};

}  // namespace uv_drc

#endif
