#include "UvDRC/UvDRC.hh"

#include <iostream>

#include "odb/db.h"
#include "odb/geom.h"
#include "stt/SteinerTreeBuilder.h"

namespace uv_drc {

void UvDRCSlewBuffer::TestFunction()
{
  std::cout << "UvDRC TestFunction called!" << std::endl;
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

  LocVec locs {};
  LocMap loc_map {};

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

stt::Tree UvDRCSlewBuffer::makeSteinerTree(LocVec& locs, LocMap& loc_map)
{
  if (locs.size() < 2) {
    throw std::runtime_error("At least two locations are required to build a Steiner Tree.");
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
    throw std::runtime_error("Buffering for fixing slew can only be called after placement.");
  }

  return resizer_->getSteinerTreeBuilder()->makeSteinerTree(
      db_network->findFlatDbNet(drvr_pin_), x, y, drvr_idx);
}

  // TODO: impl
  // Store a map of location to pin
  // Build Steiner Tree
  // Build RCTree based on Steiner Tree
  // Prepare RCTree nodes for buffer insertion

  // if (pin_count >= 2) {

  //   std::vector<int> x;
  //   std::vector<int> y;

  //   int drvr_idx = 0;  // The "driver_pin" or the root of the Steiner tree.
  //   for (int i = 0; i < pin_count; i++) {
  //     const PinLoc& pinloc = pin_locs[i];

  //     if (pinloc.pin == drvr_pin_) {
  //       drvr_idx = i;  // drvr_index is needed by flute.
  //     }

  //     x.push_back(pinloc.loc.x());
  //     y.push_back(pinloc.loc.y());

  //     // Track that all our pins are placed.
  //     is_placed &= db_network->isPlaced(pinloc.pin);

  //     // Flute may reorder the input points, so it takes some unravelling
  //     // to find the mapping back to the original pins. The complication is
  //     // that multiple pins can occupy the same location.
  //     tree->locAddPin(pinloc.loc, pinloc.pin);
  //   }
  //   if (is_placed) {
  //     stt::Tree ftree = stt_builder_->makeSteinerTree(db_net, x, y,
  //     drvr_idx);

  //     tree->setTree(ftree);
  //     tree->createSteinerPtToPinMap();
  //     return tree;
  //   }
  // }
  // delete tree;

}  // namespace uv_drc
