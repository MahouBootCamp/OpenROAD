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
