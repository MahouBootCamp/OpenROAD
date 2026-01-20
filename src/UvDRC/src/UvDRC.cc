#include "UvDRC/UvDRC.hh"

#include <iostream>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include "odb/db.h"
#include "odb/geom.h"
#include "stt/SteinerTreeBuilder.h"
#include "utl/Logger.h"

namespace uv_drc {

void UvDRCSlewBuffer::TestFunction()
{
  std::cout << "UvDRC TestFunction called!" << std::endl;
  std::vector<int> x = {0, 6, 4, 4};
  std::vector<int> y = {4, 4, 6, 2};
  stt::SteinerTreeBuilder builder{nullptr, nullptr};
  stt::Tree tree = builder.makeSteinerTree(x, y, 0);
  auto logger = utl::Logger{"UvDRCTestLog.txt", "UvDRCTestMetric.txt"};
  tree.printTree(&logger);
}

// LocEqual implementation
bool LocEqual::operator()(const odb::Point& loc1, const odb::Point& loc2) const
{
  return loc1.x() == loc2.x() && loc1.y() == loc2.y();
}

// LocHash implementation
size_t LocHash::operator()(const odb::Point& loc) const
{
  size_t seed = loc.x() * 31 + loc.y();
  seed = HashCombine(seed, std::hash<int>()(loc.x()));
  seed = HashCombine(seed, std::hash<int>()(loc.y()));
  return seed;
}

std::size_t LocHash::HashCombine(std::size_t seed, std::size_t value) const
{
  return seed ^ (value + 0x9e3779b9 + (seed << 6) + (seed >> 2));
}

std::tuple<LocVec, LocMap> UvDRCSlewBuffer::InitNetConnections()
{
  auto network = resizer_->network();
  // auto sdc_network = network->sdcNetwork();
  auto db_network = resizer_->getDbNetwork();
  odb::dbNet* db_net = db_network->findFlatDbNet(drvr_pin_);
  auto net = db_network->dbToSta(db_net);

  LocVec locs{};
  LocMap loc_map{};

  auto pin_iter = network->connectedPinIterator(net);
  while (pin_iter->hasNext()) {
    auto pin = pin_iter->next();
    odb::dbITerm* iterm;
    odb::dbBTerm* bterm;
    odb::dbModITerm* mod_iterm;
    db_network->staToDb(pin, iterm, bterm, mod_iterm);
    if (iterm || bterm) {
      odb::Point loc = db_network->location(pin);
      locs.push_back(loc);
      loc_map[loc] = pin;
    }
  }
  delete pin_iter;

  return {locs, loc_map};
}

stt::Tree UvDRCSlewBuffer::MakeSteinerTree(LocVec& locs, LocMap& loc_map)
{
  if (locs.size() < 2) {
    throw std::runtime_error(
        "At least two locations are required to build a Steiner Tree.");
  }

  std::vector<int> x;
  std::vector<int> y;
  std::size_t drvr_idx = 0;

  bool is_placed = true;
  auto db_network = resizer_->getDbNetwork();

  for (std::size_t i = 0; i != locs.size(); i++) {
    const odb::Point& loc = locs[i];
    x.push_back(loc.x());
    y.push_back(loc.y());
    auto pin = loc_map[loc];
    if (pin == drvr_pin_) {
      drvr_idx = i;
    }

    is_placed &= db_network->isPlaced(pin);
  }

  if (!is_placed) {
    throw std::runtime_error(
        "Buffering for fixing slew can only be called after placement.");
  }

  return resizer_->getSteinerTreeBuilder()->makeSteinerTree(
      db_network->findFlatDbNet(drvr_pin_), x, y, drvr_idx);
}

void UvDRCSlewBuffer::BuildRCTree(stt::Tree& tree,
                                  LocVec& locs,
                                  LocMap& loc_map)
{
  std::unordered_map<odb::Point, RCTreeNodePtr, LocHash, LocEqual>
      loc_node_map{};

  auto InitNode = [&](odb::Point loc) -> RCTreeNodePtr {
    if (loc_node_map.find(loc) != loc_node_map.end()) {
      return loc_node_map[loc];
    }

    RCTreeNodePtr node = nullptr;
    auto pin_iter = loc_map.find(loc);
    if (pin_iter != loc_map.end()) {
      const sta::Pin* pin = pin_iter->second;
      if (pin == drvr_pin_) {
        node = std::make_shared<DrivNode>(loc, pin);
        root_ = node;
      } else {
        node = std::make_shared<LoadNode>(loc, pin);
      }
    } else {
      node = std::make_shared<JuncNode>(loc);
    }
    loc_node_map[loc] = node;
    return node;
  };

  // Init load, driv & junctions first
  for (std::size_t i = 0; i != tree.branchCount(); i++) {
    auto& branch = tree.branch[i];
    odb::Point loc{branch.x, branch.y};
    auto& ref_branch = tree.branch[branch.n];
    odb::Point ref_loc{ref_branch.x, ref_branch.y};

    auto node = InitNode(loc);
    auto ref_node = InitNode(ref_loc);
    if (ref_node->Type() != RCTreeNodeType::JUNCTION) {
      auto new_ref_node = std::make_shared<JuncNode>(ref_loc);
      loc_node_map[ref_loc] = new_ref_node;
      if (ref_node->Type() == RCTreeNodeType::DRIVER) {
        ref_node->AddDownstreamNode(new_ref_node);
      } else if (ref_node->Type() == RCTreeNodeType::LOAD) {
        new_ref_node->AddDownstreamNode(ref_node);
      }
    }
    ref_node->AddDownstreamNode(node);
  }
}

void UvDRCSlewBuffer::PrepareBufferSlots()
{
  // TODO: IMPL
  // TODO: Calculate the H/V length between 2 minimum buffers
  double h_length = 1.0;  // in meters
  double v_length = 1.0;  // in meters

  auto d_nodes = root_->DownstreamNodes();
  for (auto& d_node : d_nodes) {
    PrepareBufferSlotsHelper(root_, d_node, h_length, v_length);
  }
}

void UvDRCSlewBuffer::PrepareBufferSlotsHelper(RCTreeNodePtr& u,
                                               RCTreeNodePtr& d,
                                               double h_length,
                                               double v_length)
{
  auto u_loc = u->Location();
  auto d_loc = d->Location();
  int delta_x = std::abs(u_loc.x() - d_loc.x());
  int delta_y = std::abs(u_loc.y() - d_loc.y());

  int h_step = 0;
  int v_step = 0;

  if (delta_x == 0) {
    v_step = resizer_->metersToDbu(v_length);
  } else if (delta_y == 0) {
    h_step = resizer_->metersToDbu(h_length);
  } else {
    double slope = static_cast<double>(delta_y) / static_cast<double>(delta_x);
    double newHLength = (v_length * h_length) / (v_length + slope * h_length);
    h_step = resizer_->metersToDbu(newHLength);
    v_step = resizer_->metersToDbu(slope * newHLength);
  }

  if (delta_x > h_step && delta_y > v_step) {
    bool is_right = (u_loc.x() > d_loc.x());
    bool is_up = (u_loc.y() > d_loc.y());
    RCTreeNodePtr prevWireNode = nullptr;
    auto LocIsOK = [&](int x, int y) -> bool {
      return (((is_right && x <= u_loc.x()) || (!is_right && x >= u_loc.x()))
              && ((is_up && y <= u_loc.y()) || (!is_up && y >= u_loc.y())));
    };
    int next_x = is_right ? d_loc.x() + h_step : d_loc.x() - h_step;
    int next_y = is_up ? d_loc.y() + v_step : d_loc.y() - v_step;
    while (LocIsOK(next_x, next_y)) {
      odb::Point buf_loc{next_x, next_y};
      RCTreeNodePtr wireNode = std::make_shared<WireNode>(buf_loc);
      if (prevWireNode != nullptr) {
        wireNode->AddDownstreamNode(prevWireNode);
      } else {
        wireNode->AddDownstreamNode(d);
      }
      prevWireNode = wireNode;

      next_x = is_right ? next_x + h_step : next_x - h_step;
      next_y = is_up ? next_y + v_step : next_y - v_step;
    }
    u->RemoveDownstreamNode(d);
    u->AddDownstreamNode(prevWireNode);
  }

  auto next_d_nodes = d->DownstreamNodes();
  for (auto& next_d_node : next_d_nodes) {
    PrepareBufferSlotsHelper(d, next_d_node, h_length, v_length);
  }
}

void UvDRCSlewBuffer::Run()
{
  if (root_ != nullptr) {
    throw std::runtime_error("UvDRCSlewBuffer object should run once!");
  }

  {
    auto [locs, loc_map] = InitNetConnections();
    stt::Tree steiner_tree = MakeSteinerTree(locs, loc_map);
    steiner_tree.printTree(resizer_->logger());
    BuildRCTree(steiner_tree, locs, loc_map);
    PrepareBufferSlots();
  }
}

}  // namespace uv_drc
