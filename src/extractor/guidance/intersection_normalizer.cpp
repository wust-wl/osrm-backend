#include "extractor/guidance/intersection_normalizer.hpp"
#include "extractor/guidance/toolkit.hpp"
#include "util/bearing.hpp"
#include "util/guidance/toolkit.hpp"

#include "extractor/geojson_debug_policies.hpp"
#include "util/geojson_debug_logger.hpp"
#include "util/geojson_debug_policies.hpp"

using osrm::util::angularDeviation;

namespace osrm
{
namespace extractor
{
namespace guidance
{

IntersectionNormalizer::IntersectionNormalizer(
    const util::NodeBasedDynamicGraph &node_based_graph,
    const std::vector<extractor::QueryNode> &node_coordinates,
    const util::NameTable &name_table,
    const SuffixTable &street_name_suffix_table,
    const IntersectionGenerator &intersection_generator)
    : node_based_graph(node_based_graph), name_table(name_table),
      street_name_suffix_table(street_name_suffix_table),
      intersection_generator(intersection_generator),
      mergable_road_detector(node_based_graph,
                             node_coordinates,
                             intersection_generator,
                             intersection_generator.GetCoordinateExtractor())
{
}

std::pair<IntersectionShape, std::vector<std::pair<EdgeID, EdgeID>>> IntersectionNormalizer::
operator()(const NodeID node_at_intersection, IntersectionShape intersection) const
{
    std::cout << "Processing:\n";
    for( auto road : intersection )
        std::cout << "\t" << road.eid << " " << road.bearing << std::endl;
    const auto intersection_copy = intersection;;
    auto merged_shape_and_merges =
        MergeSegregatedRoads(node_at_intersection, std::move(intersection));
    if (!merged_shape_and_merges.second.empty())
    {
        IntersectionShape reduced_shape;
        for (auto road : intersection_copy)
        {
            if (merged_shape_and_merges.second.end() !=
                std::find_if(merged_shape_and_merges.second.begin(),
                             merged_shape_and_merges.second.end(),
                             [road](const auto &val) {
                                 return road.eid == val.first || road.eid == val.second;
                             }))
                reduced_shape.push_back(road);
        }
        util::ScopedGeojsonLoggerGuard<extractor::IntersectionShapePrinter>::Write(
            node_at_intersection, reduced_shape);
    }

    merged_shape_and_merges.first = AdjustBearingsForMergeAtDestination(
        node_at_intersection, std::move(merged_shape_and_merges.first));
    return merged_shape_and_merges;
}

bool IntersectionNormalizer::CanMerge(const NodeID intersection_node,
                                      const IntersectionShape &intersection,
                                      std::size_t first_index,
                                      std::size_t second_index) const
{
    BOOST_ASSERT(((first_index + 1) % intersection.size()) == second_index);

    // call wrapper to capture intersection_node and intersection
    const auto mergable = [this, intersection_node, &intersection](const std::size_t left_index,
                                                                   const std::size_t right_index) {
        return InnerCanMerge(intersection_node, intersection, left_index, right_index);
    };

    /*
     * Merging should never depend on order/never merge more than two roads. To ensure that we don't
     * merge anything that is impacted by neighboring roads (e.g. three roads of the same name as in
     * parking lots/border checkpoints), we check if the neigboring roads would be merged as well.
     * In that case, we cannot merge, since we would end up merging multiple items together
     */
    if (mergable(first_index, second_index))
    {
        const auto is_distinct_merge =
            !mergable(second_index, (second_index + 1) % intersection.size()) &&
            !mergable((first_index + intersection.size() - 1) % intersection.size(), first_index) &&
            !mergable(second_index,
                      (first_index + intersection.size() - 1) % intersection.size()) &&
            !mergable(first_index, (second_index + 1) % intersection.size());
        return is_distinct_merge;
    }
    else
        return false;
}

// Checks for mergability of two ways that represent the same intersection. For further
// information see interface documentation in header.
bool IntersectionNormalizer::InnerCanMerge(const NodeID node_at_intersection,
                                           const IntersectionShape &intersection,
                                           std::size_t first_index,
                                           std::size_t second_index) const
{
    const auto &first_data = node_based_graph.GetEdgeData(intersection[first_index].eid);
    const auto &second_data = node_based_graph.GetEdgeData(intersection[second_index].eid);

    // don't merge on degree two, since it's most likely a bollard/traffic light or a round way
    if (intersection.size() <= 2)
        return false;

    // only merge named ids
    if (first_data.name_id == EMPTY_NAMEID || second_data.name_id == EMPTY_NAMEID)
        return false;

    // need to be same name
    if (util::guidance::requiresNameAnnounced(
            first_data.name_id, second_data.name_id, name_table, street_name_suffix_table))
        return false;
    // needs to be symmetrical for names
    if (util::guidance::requiresNameAnnounced(
            second_data.name_id, first_data.name_id, name_table, street_name_suffix_table))
        return false;

    return mergable_road_detector.CanMergeRoad(
            node_at_intersection, intersection[first_index], intersection[second_index]);
}

/*
 * Segregated Roads often merge onto a single intersection.
 * While technically representing different roads, they are
 * often looked at as a single road.
 * Due to the merging, turn Angles seem off, wenn we compute them from the
 * initial positions.
 *
 *         b<b<b<b(1)<b<b<b
 * aaaaa-b
 *         b>b>b>b(2)>b>b>b
 *
 * Would be seen as a slight turn going fro a to (2). A Sharp turn going from
 * (1) to (2).
 *
 * In cases like these, we megre this segregated roads into a single road to
 * end up with a case like:
 *
 * aaaaa-bbbbbb
 *
 * for the turn representation.
 * Anything containing the first u-turn in a merge affects all other angles
 * and is handled separately from all others.
 */
std::pair<IntersectionShape, std::vector<std::pair<EdgeID, EdgeID>>>
IntersectionNormalizer::MergeSegregatedRoads(const NodeID intersection_node,
                                             IntersectionShape intersection) const
{
    const auto getRight = [&](std::size_t index) {
        return (index + intersection.size() - 1) % intersection.size();
    };

    // we only merge small angles. If the difference between both is large, we are looking at a
    // bearing leading north. Such a bearing cannot be handled via the basic average. In this
    // case we actually need to shift the bearing by half the difference.
    const auto aroundZero = [](const double first, const double second) {
        return (std::max(first, second) - std::min(first, second)) >= 180;
    };

    // find the angle between two other angles
    const auto combineAngles = [aroundZero](const double first, const double second) {
        if (!aroundZero(first, second))
            return .5 * (first + second);
        else
        {
            const auto offset = angularDeviation(first, second);
            auto new_angle = std::max(first, second) + .5 * offset;
            if (new_angle >= 360)
                return new_angle - 360;
            return new_angle;
        }
    };

    // This map stores for all edges that participated in a merging operation in which edge id they
    // end up in the end. We only store what we have merged into other edges.
    std::vector<std::pair<EdgeID, EdgeID>> merging_map;

    const auto merge = [this, combineAngles, &merging_map](const IntersectionShapeData &first,
                                                           const IntersectionShapeData &second) {
        IntersectionShapeData result =
            !node_based_graph.GetEdgeData(first.eid).reversed ? first : second;
        result.bearing = combineAngles(first.bearing, second.bearing);
        BOOST_ASSERT(0 <= result.bearing && result.bearing < 360.0);
        // the other ID
        const auto merged_from = result.eid == first.eid ? second.eid : first.eid;
        BOOST_ASSERT(
            std::find_if(merging_map.begin(), merging_map.end(), [merged_from](const auto pair) {
                return pair.first == merged_from;
            }) == merging_map.end());
        merging_map.push_back(std::make_pair(merged_from, result.eid));
        return result;
    };

    if (intersection.size() <= 1)
        return std::make_pair(intersection, merging_map);

    bool merged = false;
    const auto intersection_copy = intersection;
    // check for merges including the basic u-turn
    // these result in an adjustment of all other angles. This is due to how these angles are
    // perceived. Considering the following example:
    //
    //   c   b
    //     Y
    //     a
    //
    // coming from a to b (given a road that splits at the fork into two one-ways), the turn is not
    // considered as a turn but rather as going straight.
    // Now if we look at the situation merging:
    //
    //  a     b
    //    \ /
    // e - + - d
    //     |
    //     c
    //
    // With a,b representing the same road, the intersection itself represents a classif for way
    // intersection so we handle it like
    //
    //   (a),b
    //      |
    // e -  + - d
    //      |
    //      c
    //
    // To be able to consider this adjusted representation down the line, we merge some roads.
    // If the merge occurs at the u-turn edge, we need to adjust all angles, though, since they are
    // with respect to the now changed perceived location of a. If we move (a) to the left, we add
    // the difference to all angles. Otherwise we subtract it.
    bool merged_first = false;
    // these result in an adjustment of all other angles
    if (CanMerge(intersection_node, intersection, intersection.size() - 1, 0))
    {
        merged_first = true;
        // moving `a` to the left
        intersection[0] = merge(intersection.front(), intersection.back());
        // FIXME if we have a left-sided country, we need to switch this off and enable it
        // below
        intersection.pop_back();
        merged = true;
    }
    else if (CanMerge(intersection_node, intersection, 0, 1))
    {
        merged_first = true;
        intersection[0] = merge(intersection.front(), intersection[1]);
        intersection.erase(intersection.begin() + 1);
        merged = true;
    }

    // a merge including the first u-turn requires an adjustment of the turn angles
    // therefore these are handled prior to this step
    for (std::size_t index = 2; index < intersection.size(); ++index)
    {
        if (CanMerge(intersection_node, intersection, getRight(index), index))
        {
            intersection[getRight(index)] =
                merge(intersection[getRight(index)], intersection[index]);
            intersection.erase(intersection.begin() + index);
            --index;
            merged = true;
        }
    }
    return std::make_pair(intersection, merging_map);
}

// OSM can have some very steep angles for joining roads. Considering the following intersection:
//        x
//        |
//        v __________c
//       /
// a ---d
//       \ __________b
//
// with c->d as a oneway
// and d->b as a oneway, the turn von x->d is actually a turn from x->a. So when looking at the
// intersection coming from x, we want to interpret the situation as
//           x
//           |
// a __ d __ v__________c
//      |
//      |_______________b
//
// Where we see the turn to `d` as a right turn, rather than going straight.
// We do this by adjusting the local turn angle at `x` to turn onto `d` to be reflective of this
// situation, where `v` would be the node at the intersection.
IntersectionShape
IntersectionNormalizer::AdjustBearingsForMergeAtDestination(const NodeID node_at_intersection,
                                                            IntersectionShape intersection) const
{
    // nothing to do for dead ends
    if (intersection.size() <= 1)
        return intersection;

    // we don't adjust any road that is longer than 30 meters (between centers of intersections),
    // since the road is probably too long otherwise to impact perception.
    const double constexpr PRUNING_DISTANCE = 30;
    // never adjust u-turns
    for (std::size_t index = 0; index < intersection.size(); ++index)
    {
        auto &road = intersection[index];
        // only consider roads that are close
        if (road.segment_length > PRUNING_DISTANCE)
            continue;

        // to find out about the above situation, we need to look at the next intersection (at d in
        // the example). If the initial road can be merged to the left/right, we are about to adjust
        // the angle.
        const auto next_intersection_along_road = intersection_generator.ComputeIntersectionShape(
            node_based_graph.GetTarget(road.eid), node_at_intersection);

        if (next_intersection_along_road.size() <= 1)
            continue;

        const auto node_at_next_intersection = node_based_graph.GetTarget(road.eid);

        const auto adjustAngle = [](double angle, double offset) {
            angle += offset;
            if (angle > 360)
                return angle - 360.;
            else if (angle < 0)
                return angle + 360.;
            return angle;
        };

        const auto range = node_based_graph.GetAdjacentEdgeRange(node_at_next_intersection);
        if (range.size() <= 1)
            continue;

        // the order does not matter
        const auto get_offset = [](const IntersectionShapeData &lhs,
                                   const IntersectionShapeData &rhs) {
            return 0.5 * angularDeviation(lhs.bearing, rhs.bearing);
        };

        // When offsetting angles in our turns, we don't want to get past the next turn. This
        // function simply limits an offset to be at most half the distance to the next turn in the
        // offfset direction
        const auto get_corrected_offset = [](
            const double offset,
            const IntersectionShapeData &road,
            const IntersectionShapeData &next_road_in_offset_direction) {
            const auto offset_limit =
                angularDeviation(road.bearing, next_road_in_offset_direction.bearing);
            // limit the offset with an additional buffer
            return (offset + MAXIMAL_ALLOWED_NO_TURN_DEVIATION > offset_limit) ? 0.5 * offset_limit
                                                                               : offset;
        };

        // check if the u-turn edge at the next intersection could be merged to the left/right. If
        // this is the case and the road is not far away (see previous distance check), if
        // influences the perceived angle.
        if (CanMerge(node_at_next_intersection, next_intersection_along_road, 0, 1))
        {
            const auto offset =
                get_offset(next_intersection_along_road[0], next_intersection_along_road[1]);

            const auto corrected_offset = get_corrected_offset(
                offset,
                road,
                intersection[(intersection.size() + index - 1) % intersection.size()]);
            // at the target intersection, we merge to the right, so we need to shift the current
            // angle to the left
            road.bearing = adjustAngle(road.bearing, -corrected_offset);
        }
        else if (CanMerge(node_at_next_intersection,
                          next_intersection_along_road,
                          next_intersection_along_road.size() - 1,
                          0))
        {
            const auto offset =
                get_offset(next_intersection_along_road[0],
                           next_intersection_along_road[next_intersection_along_road.size() - 1]);

            const auto corrected_offset =
                get_corrected_offset(offset, road, intersection[(index + 1) % intersection.size()]);
            // at the target intersection, we merge to the left, so we need to shift the current
            // angle to the right
            road.bearing = adjustAngle(road.bearing, corrected_offset);
        }
    }
    return intersection;
}

} // namespace guidance
} // namespace extractor
} // namespace osrm
