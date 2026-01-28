#include "UvDRC/UvDRC.hh"

#include <algorithm>
#include <cstddef>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include "odb/db.h"
#include "odb/geom.h"
#include "sta/MinMax.hh"
#include "stt/SteinerTreeBuilder.h"
#include "utl/Logger.h"

namespace uv_drc {

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

void RCTreeNode::DebugPrint(int indent, utl::Logger* logger)
{
  std::stringstream os;
  {
    std::string indent_str(indent, ' ');
    os << "Node Type: " << indent_str;
  }
  switch (type_) {
    case RCTreeNodeType::LOAD:
      os << "LOAD";
      break;
    case RCTreeNodeType::WIRE:
      os << "WIRE";
      break;
    case RCTreeNodeType::JUNCTION:
      os << "JUNCTION";
      break;
    case RCTreeNodeType::DRIVER:
      os << "DRIVER";
      break;
    default:
      os << "NONE";
      break;
  }
  os << ", Location: (" << loc_.x() << ", " << loc_.y() << ")";
  logger->report(os.str());
  auto d_nodes = DownstreamNodes();
  for (const auto& d : d_nodes) {
    d->DebugPrint(indent + 2, logger);
  }
}

void UvDRCSlewBuffer::InitBufferCandidates()
{
  if (!buffer_candidates_.empty()) {
    return;
  }

  for (auto b : resizer_->buffer_fast_sizes_) {
    sta::LibertyPort *in, *out;
    b->bufferPorts(in, out);
    float drive_resistance = out->driveResistance();
    float input_cap = in->capacitance();
    BufferCandidate candidate{b, input_cap, drive_resistance};
    buffer_candidates_.push_back(candidate);
  }
  // Sort by input capacitance ascendingly
  std::sort(buffer_candidates_.begin(),
            buffer_candidates_.end(),
            [](const BufferCandidate& a, const BufferCandidate& b) {
              return a.input_cap < b.input_cap;
            });
}

std::tuple<LocVec, PinVec> UvDRCSlewBuffer::InitNetConnections(
    const sta::Pin* drvr_pin)
{
  auto network = resizer_->network();
  // auto sdc_network = network->sdcNetwork();
  auto db_network = resizer_->getDbNetwork();
  odb::dbNet* db_net = db_network->findFlatDbNet(drvr_pin);
  auto net = db_network->dbToSta(db_net);

  LocVec locs{};
  PinVec pins{};
  locs.push_back(db_network->location(drvr_pin));
  pins.push_back(drvr_pin);

  auto pin_iter = network->connectedPinIterator(net);
  while (pin_iter->hasNext()) {
    auto pin = pin_iter->next();
    odb::dbITerm* iterm;
    odb::dbBTerm* bterm;
    odb::dbModITerm* mod_iterm;
    db_network->staToDb(pin, iterm, bterm, mod_iterm);
    if ((iterm || bterm) && pin != drvr_pin) {
      odb::Point loc = db_network->location(pin);
      locs.push_back(loc);
      pins.push_back(pin);
    }
  }
  delete pin_iter;

  return {locs, pins};
}

stt::Tree UvDRCSlewBuffer::MakeSteinerTree(const sta::Pin* drvr_pin,
                                           LocVec& locs,
                                           PinVec& pins)
{
  if (locs.size() < 2) {
    throw std::runtime_error(
        "At least two locations are required to build a Steiner Tree.");
  }

  std::vector<int> x;
  std::vector<int> y;

  bool is_placed = true;
  auto db_network = resizer_->getDbNetwork();

  for (std::size_t i = 0; i != locs.size(); i++) {
    const odb::Point& loc = locs[i];
    x.push_back(loc.x());
    y.push_back(loc.y());
    is_placed &= db_network->isPlaced(pins[i]);
  }

  if (!is_placed) {
    throw std::runtime_error(
        "Buffering for fixing slew can only be called after placement.");
  }

  return resizer_->getSteinerTreeBuilder()->makeSteinerTree(
      db_network->findFlatDbNet(drvr_pin), x, y, 0);
}

// From SteinerTree to RCTree structure
RCTreeNodePtr UvDRCSlewBuffer::BuildRCTree(const sta::Pin* drvr_pin,
                                           stt::Tree& tree,
                                           LocVec& locs,
                                           PinVec& pins)
{
  std::size_t pin_count = static_cast<std::size_t>(tree.deg);
  std::vector<RCTreeNodePtr> pin_nodes{};
  pin_nodes.push_back(
      std::make_shared<DrivNode>(locs[0], drvr_pin));  // Driver node
  for (std::size_t i = 1; i != pin_count; i++) {
    pin_nodes.push_back(
        std::make_shared<LoadNode>(locs[i], pins[i]));  // Driver node
  }
  std::unordered_map<odb::Point, RCTreeNodePtr, LocHash, LocEqual>
      junc_node_map{};

  auto GetOrInitJuncNode = [&](odb::Point loc) -> RCTreeNodePtr {
    auto itr = junc_node_map.find(loc);
    if (itr == junc_node_map.end()) {
      RCTreeNodePtr node = std::make_shared<JuncNode>(loc);
      junc_node_map[loc] = node;
      return node;
    }
    return itr->second;
  };

  for (std::size_t i = 0; i != tree.branchCount(); i++) {
    auto& branch = tree.branch[i];
    auto node = GetOrInitJuncNode(odb::Point(branch.x, branch.y));
    if (i < pin_count) {
      if (i == 0) {
        pin_nodes[i]->AddDownstreamNode(node);
      } else {
        node->AddDownstreamNode(pin_nodes[i]);
      }
    }
    if (i == 0) continue;
    auto parent_branch = tree.branch[branch.n];
    odb::Point parent_loc{parent_branch.x, parent_branch.y};
    RCTreeNodePtr parent_node = GetOrInitJuncNode(parent_loc);
    parent_node->AddDownstreamNode(node);
  }

  pin_nodes[0]->DebugPrint(0, resizer_->logger());
  return pin_nodes[0];
}

// TODO: IMPL
void UvDRCSlewBuffer::PrepareBufferSlots(RCTreeNodePtr root,
                                         const sta::Corner* corner)
{
  auto max_wire_length = resizer_->metersToDbu(resizer_->findMaxWireLength());
  max_wire_length = std::min(
      max_wire_length,
      std::min(MaxLengthForSlew(buffer_candidates_.front().cell, corner),
               MaxLengthForCap(buffer_candidates_.front().cell, corner)));

  auto buffer_step = max_wire_length / 2;
  resizer_->logger()->report("Preparing buffer slots with buffer step: {}",
                             buffer_step);

  // Prepare buffer slots per max_wire_length
  auto d_nodes = root->DownstreamNodes();
  for (auto& d : d_nodes) {
    PrepareBufferSlotsHelper(root, d, buffer_step);
  }
  root->DebugPrint(0, resizer_->logger());
}

// TODO: IMPL
void UvDRCSlewBuffer::PrepareBufferSlotsHelper(RCTreeNodePtr u,
                                               RCTreeNodePtr d,
                                               int buffer_step)
{
  auto u_loc = u->Location();
  auto d_loc = d->Location();
  auto distance = odb::Point::manhattanDistance(u_loc, d_loc);
  if (distance > buffer_step) {
    // TODO: insert wire nodes
    // NOTE: here we divide the wire into segments of same length,
    // another way is to prepare a slot per buffer_step.
    int n = distance / buffer_step;
    int delta_x = (d_loc.x() - u_loc.x()) / (n + 1);
    int delta_y = (d_loc.y() - u_loc.y()) / (n + 1);
    RCTreeNodePtr prevWireNode = nullptr;
    for (int i = 1; i <= n; i++) {
      odb::Point wire_node_loc{d_loc.x() - i * delta_x,
                               d_loc.y() - i * delta_y};
      RCTreeNodePtr wireNode = std::make_shared<WireNode>(wire_node_loc);
      if (prevWireNode != nullptr) {
        wireNode->AddDownstreamNode(prevWireNode);
      } else {
        wireNode->AddDownstreamNode(d);
      }
      prevWireNode = wireNode;
    }
    u->AddDownstreamNode(prevWireNode);
    u->RemoveDownstreamNode(d);
  }

  auto next_d_nodes = d->DownstreamNodes();
  for (auto& next_d_node : next_d_nodes) {
    PrepareBufferSlotsHelper(d, next_d_node, buffer_step);
  }
}

// Max length
int UvDRCSlewBuffer::MaxLengthForSlew(sta::LibertyCell* buffer_cell,
                                      const sta::Corner* corner)
{
  switch (model_) {
    case UvDRCSlewModel::Alpert: {
      return MaxLengthForSlewAlpert(buffer_cell, corner);
    }
    case UvDRCSlewModel::OpenROAD: {
      return MaxLengthForSlewOpenROAD(buffer_cell, corner);
    }
    default:
      throw std::runtime_error("Unsupported UvDRCSlewModel!");
  }
}

// TODO: Impl
int UvDRCSlewBuffer::MaxLengthForSlewAlpert(sta::LibertyCell* buffer_cell,
                                            const sta::Corner* corner)
{
  throw std::invalid_argument("Alpert Model is not implemented yet!");
}

int UvDRCSlewBuffer::MaxLengthForSlewOpenROAD(sta::LibertyCell* buffer_cell,
                                              const sta::Corner* corner)
{
  // OpenROAD's RC-based slew model
  // NOTE: the original version in Rebuffer uses Lumped RC model
  // Here I modify it to use distributed RC model (a wire's Cap would be
  // distributed to both ends)

  // In this demo, we assume unit r,c are same on H & V directions
  // TODO: Consider different unit r,c on H & V directions for better estimation
  sta::LibertyPort *in, *out;
  buffer_cell->bufferPorts(in, out);

  auto max_slew = resizer_->maxInputSlew(in, corner);
  if (max_slew == sta::INF) {
    return std::numeric_limits<int>::max();
  }
  max_slew *= (1 - k_slew_margin_);  // * 0.8

  double unit_r, unit_c;
  resizer_->estimate_parasitics_->wireSignalRC(corner, unit_r, unit_c);

  double r_drvr = out->driveResistance();
  double in_cap = in->capacitance();
  double slew_rc_factor = k_openroad_slew_factor;  // 1.39
  // Solve quadratic equation to get max length
  // 0.5 r c l^2 + (r * C_Load + R_drvr * c) l + R_drvr * C_Load = Max_Slew /
  // Delay_Slew_Factor
  const double a = unit_r * unit_c / 2;
  const double b = unit_c * r_drvr + unit_r * in_cap;
  const double c = r_drvr * in_cap - max_slew / slew_rc_factor;
  const double d = b * b - 4 * a * c;
  if (d < 0) {
    resizer_->logger()->warn(
        utl::RSZ,
        2012,
        "cannot determine wire length limit implied by load "
        "slew on cell {} in corner {}",
        buffer_cell->name(),
        corner->name());
    return std::numeric_limits<int>::max();
  }

  const double max_length_meters = (-b + std::sqrt(d)) / (2 * a);

  if (max_length_meters > 1) {
    // no limit
    return std::numeric_limits<int>::max();
  }

  if (max_length_meters <= 0) {
    resizer_->logger()->warn(
        utl::RSZ,
        2013,
        "cannot determine wire length limit implied by load "
        "slew on cell {} in corner {}",
        buffer_cell->name(),
        corner->name());
    return std::numeric_limits<int>::max();
  }

  int max_length_dbu = resizer_->metersToDbu(max_length_meters);

  if (max_length_dbu == 0) {
    resizer_->logger()->warn(
        utl::RSZ,
        2014,
        "cannot determine wire length limit implied by load "
        "slew on cell {} in corner {}",
        buffer_cell->name(),
        corner->name());
    return std::numeric_limits<int>::max();
  }

  return max_length_dbu;
}

int UvDRCSlewBuffer::MaxLengthForCap(sta::LibertyCell* buffer_cell,
                                     const sta::Corner* corner)
{
  sta::LibertyPort *in, *out;
  buffer_cell->bufferPorts(in, out);

  bool cap_limit_exists;
  float cap_limit;
  out->capacitanceLimit(sta::MinMax::max(), cap_limit, cap_limit_exists);

  if (cap_limit_exists) {
    double wire_res, wire_cap;
    resizer_->estimate_parasitics_->wireSignalRC(corner, wire_res, wire_cap);
    double max_length
        = (cap_limit * (1 - k_cap_margin_) - in->capacitance()) / wire_cap;
    return std::max(0, resizer_->metersToDbu(max_length));
  }

  return std::numeric_limits<int>::max();
}

// TODO: Impl
// void UvDRCSlewBuffer::PrepareBufferSlotsHelper(RCTreeNodePtr& u,
// RCTreeNodePtr& d,
// int max_length)
// {
// auto u_loc = u->Location();
// auto d_loc = d->Location();
// int delta_x = std::abs(u_loc.x() - d_loc.x());
// int delta_y = std::abs(u_loc.y() - d_loc.y());

// int h_step = 0;
// int v_step = 0;

// if (delta_x == 0) {
//   v_step = resizer_->metersToDbu(v_length);
// } else if (delta_y == 0) {
//   h_step = resizer_->metersToDbu(h_length);
// } else {
//   double slope = static_cast<double>(delta_y) / static_cast<double>(delta_x);
//   double newHLength = (v_length * h_length) / (v_length + slope * h_length);
//   h_step = resizer_->metersToDbu(newHLength);
//   v_step = resizer_->metersToDbu(slope * newHLength);
// }

// if (delta_x > h_step && delta_y > v_step) {
//   bool is_right = (u_loc.x() > d_loc.x());
//   bool is_up = (u_loc.y() > d_loc.y());
//   RCTreeNodePtr prevWireNode = nullptr;
//   auto LocIsOK = [&](int x, int y) -> bool {
//     return (((is_right && x <= u_loc.x()) || (!is_right && x >= u_loc.x()))
//             && ((is_up && y <= u_loc.y()) || (!is_up && y >= u_loc.y())));
//   };
//   int next_x = is_right ? d_loc.x() + h_step : d_loc.x() - h_step;
//   int next_y = is_up ? d_loc.y() + v_step : d_loc.y() - v_step;
//   while (LocIsOK(next_x, next_y)) {
//     odb::Point buf_loc{next_x, next_y};
//     RCTreeNodePtr wireNode = std::make_shared<WireNode>(buf_loc);
//     if (prevWireNode != nullptr) {
//       wireNode->AddDownstreamNode(prevWireNode);
//     } else {
//       wireNode->AddDownstreamNode(d);
//     }
//     prevWireNode = wireNode;

//     next_x = is_right ? next_x + h_step : next_x - h_step;
//     next_y = is_up ? next_y + v_step : next_y - v_step;
//   }
//   u->RemoveDownstreamNode(d);
//   u->AddDownstreamNode(prevWireNode);
// }

// auto next_d_nodes = d->DownstreamNodes();
// for (auto& next_d_node : next_d_nodes) {
//   PrepareBufferSlotsHelper(d, next_d_node, h_length, v_length);
// }
// }

void UvDRCSlewBuffer::Run(const sta::Pin* drvr_pin,
                          const sta::Corner* corner,
                          int max_cap)
{
  InitBufferCandidates();

  {
    auto [locs, loc_map] = InitNetConnections(drvr_pin);
    auto steiner_tree = MakeSteinerTree(drvr_pin, locs, loc_map);

    steiner_tree.printTree(resizer_->logger());

    auto rc_tree = BuildRCTree(drvr_pin, steiner_tree, locs, loc_map);

    PrepareBufferSlots(rc_tree, corner);
  }
}

}  // namespace uv_drc
