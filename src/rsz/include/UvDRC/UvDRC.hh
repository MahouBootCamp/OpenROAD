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
#include <unordered_map>
#include "sta/Corner.hh"

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

using LocMap
    = std::unordered_map<odb::Point, const sta::Pin*, LocHash, LocEqual>;
using LocVec = std::vector<odb::Point>;

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
  virtual void RemoveDownstreamNode(RCTreeNodePtr ptr) = 0;
  virtual std::vector<RCTreeNodePtr> DownstreamNodes() = 0;

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
  void RemoveDownstreamNode(RCTreeNodePtr ptr) override
  {
    throw std::runtime_error("LoadNode cannot have downstream nodes.");
  }
  std::vector<RCTreeNodePtr> DownstreamNodes() override { return {}; }

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
  std::vector<RCTreeNodePtr> DownstreamNodes() override { return children_; }
  void RemoveDownstreamNode(RCTreeNodePtr ptr) override
  {
    if (auto itr = std::find(children_.begin(), children_.end(), ptr);
        itr != children_.end()) {
      throw std::runtime_error(
          "Remove non-matching downstream node from Junc node.");
    }
    children_.erase(std::remove(children_.begin(), children_.end(), ptr),
                    children_.end());
  }

 private:
  std::vector<RCTreeNodePtr> children_;
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
  void TestFunction();
  void Run(const sta::Pin* drvr_pin, const sta::Corner* corner, int max_cap);

 private:
  struct BufferCandidate
  {
    sta::LibertyCell* cell;
    float input_cap;
    float drive_resistance;
  };

 private:
  void InitBufferCandidates();

  std::tuple<LocVec, LocMap> InitNetConnections(const sta::Pin* drvr_pin);
  stt::Tree MakeSteinerTree(const sta::Pin* drvr_pin,
                            LocVec& locs,
                            LocMap& loc_map);
  RCTreeNodePtr BuildRCTree(const sta::Pin* drvr_pin,
                            stt::Tree& tree,
                            LocVec& locs,
                            LocMap& loc_map);
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

 private:
  rsz::Resizer* resizer_;
  // const sta::Pin* drvr_pin_;
  // const sta::Corner* corner_;
  // int max_cap_;
  UvDRCSlewModel model_;

  std::vector<BufferCandidate> buffer_candidates_;
  // RCTreeNodePtr root_ = nullptr;

  static constexpr float k_slew_margin_ = 0.2f;  // 20%
  static constexpr float k_openroad_slew_factor = 1.39;
};

}  // namespace uv_drc

#endif
