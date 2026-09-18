// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_rule_seed.cpp
 * @brief Imported OSM features become seed shapes a rule can read
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * The geometry half of this is nearly free -- `Building::footprint` is a CCW
 * outer ring and `Building::holes` are CW inner rings, which is already
 * `shape_from_rings()`'s contract. So these tests spend their effort on the
 * half that is not free: the ATTRIBUTES, and whether a rule can actually read
 * them.
 *
 * Every test that claims a rule can read an attribute proves it by RUNNING a
 * rule that reads it, not by inspecting the map. A seed whose attributes are
 * all present and which no rule can reach would pass the weaker test.
 */

#include "framework.hpp"

#include "osm/mesh_builder.hpp"
#include "osm/rule_seed.hpp"
#include "procgen/rules/parser.hpp"
#include "procgen/rules/registry.hpp"

#include <string>

using namespace stratum::osm;
using namespace stratum::procgen::rules;

namespace {

/// A 10 x 8 rectangular building, counter-clockwise as the parser produces
[[nodiscard]] Building make_building() {
    Building b;
    b.osm_id = 4242;
    b.footprint = {{0.0, 0.0}, {10.0, 0.0}, {10.0, 8.0}, {0.0, 8.0}};
    b.height = 12.5f;
    b.levels = 4;
    b.type = BuildingType::Apartments;
    b.roof_type = RoofType::Hipped;
    return b;
}

/// Run @p source against @p seed through the same table the editor uses
[[nodiscard]] GenerationResult run_on(const Shape& seed, const std::string& source) {
    const ParseResult parsed = parse(source, "seed.srl");
    CHECK_EQ(parsed.render_all(source), std::string{});
    return generate(parsed.file, seed, GenerationOptions{}, &full_operations(), &full_functions());
}

[[nodiscard]] bool logged(const GenerationResult& result, const std::string& needle) {
    for (const std::string& line : result.log) {
        if (line.find(needle) != std::string::npos) return true;
    }
    return false;
}

} // namespace

// ============================================================================
// Geometry
// ============================================================================

TEST(RuleSeed, a_building_footprint_becomes_a_shape_with_the_same_area) {
    const Shape seed = seed_from_building(make_building());
    CHECK_EQ(seed.geometry.faces.size(), size_t{1});
    CHECK_NEAR(geometry_area(seed.geometry), 80.0, 1e-9);
    // Flat on the ground: a footprint, not a solid. A rule gives it height.
    CHECK_NEAR(seed.scope.size.y, 0.0, 1e-12);
}

TEST(RuleSeed, a_courtyard_building_keeps_its_hole) {
    Building b = make_building();
    b.footprint = {{0.0, 0.0}, {20.0, 0.0}, {20.0, 20.0}, {0.0, 20.0}};
    // Clockwise, as ParsedOSMData holds an inner ring.
    b.holes = {{{6.0, 6.0}, {6.0, 14.0}, {14.0, 14.0}, {14.0, 6.0}}};

    const Shape seed = seed_from_building(b);
    CHECK_EQ(seed.geometry.faces.size(), size_t{1});
    CHECK_EQ(seed.geometry.faces[0].holes.size(), size_t{1});
    // 400 - 64. A hole left wound the wrong way would ADD its area instead.
    CHECK_NEAR(geometry_area(seed.geometry), 336.0, 1e-9);
}

TEST(RuleSeed, a_footprint_wound_the_wrong_way_is_still_seeded_upward) {
    // An importer, or a hand-edited file, can hand back a clockwise outer ring.
    // shape_from_rings fixes the winding rather than trusting it, and this is
    // the test that says the seeding path inherits that.
    Building b = make_building();
    b.footprint = {{0.0, 0.0}, {0.0, 8.0}, {10.0, 8.0}, {10.0, 0.0}};

    const Shape seed = seed_from_building(b);
    CHECK_NEAR(geometry_area(seed.geometry), 80.0, 1e-9);

    const GenerationResult result = run_on(seed, "@start\nrule M { extrude(3.0); }\n");
    CHECK_TRUE(result.ok());
    CHECK_EQ(result.terminals.size(), size_t{1});
    // A footprint seeded face-down extrudes into a solid of NEGATIVE volume.
    if (!result.terminals.empty()) {
        CHECK_NEAR(geometry_volume(result.terminals[0].geometry), 240.0, 1e-9);
    }
}

TEST(RuleSeed, a_degenerate_footprint_yields_a_seed_with_no_geometry) {
    // Dropped seeds would renumber every seed after them, which changes what
    // the whole city generates. So a bad footprint still produces a seed, and
    // still carries its id so a caller can say which feature it was.
    Building b = make_building();
    b.footprint = {{0.0, 0.0}, {1.0, 0.0}};

    const Shape seed = seed_from_building(b);
    CHECK_TRUE(seed.geometry.faces.empty());
    const auto found = seed.attributes.find(kSeedOsmId);
    CHECK_TRUE(found != seed.attributes.end());
}

// ============================================================================
// Alignment with the rest of the scene
// ============================================================================

TEST(RuleSeed, a_seeded_building_lands_where_the_imported_mesh_does) {
    // THE TEST THIS FILE MOST NEEDED AND DID NOT HAVE.
    //
    // shape_from_rings() lifts a 2D point to {p.x, y, p.y}. Every mesh builder
    // in src/osm and src/osm/road lifts it to {p.x, height, -p.y}, without
    // exception -- the 2D plane is x-east/y-north and world space is Y-up with
    // north along -z.
    //
    // Seeding without that negation mirrors every generated building about the
    // z axis. At the scale of one building it looks like a building; at the
    // scale of a city the generated blocks sit ON the roads instead of beside
    // them, which is how it was actually spotted.
    //
    // So this asserts against MeshBuilder rather than against a literal: the
    // two are required to agree, and if either convention is ever changed the
    // other has to change with it.
    Building b = make_building();
    // Deliberately not symmetric about either axis. A footprint that is
    // symmetric in y survives the mirror unchanged and proves nothing.
    b.footprint = {{2.0, 1.0}, {12.0, 1.0}, {12.0, 5.0}, {7.0, 9.0}, {2.0, 5.0}};
    b.height = 8.0f;
    // Flat, so MeshBuilder raises a plain box and the rule's bare extrude is
    // comparable. The fixture is Hipped, and MeshBuilder would put a roof on
    // top of it -- a real difference, and not the one under test here.
    b.roof_type = RoofType::Flat;

    const stratum::Mesh imported = MeshBuilder::build_building_mesh(b);
    CHECK_FALSE(imported.vertices.empty());

    const Shape seed = seed_from_building(b);
    const GenerationResult generated = run_on(seed, "@start\nrule M { extrude(8.0); }\n");
    CHECK_TRUE(generated.ok());
    CHECK_EQ(generated.terminals.size(), size_t{1});
    if (generated.terminals.empty()) return;

    const stratum::Mesh built = generated.build_mesh();
    CHECK_FALSE(built.vertices.empty());

    // Same footprint, same height, so the same world bounding box.
    CHECK_NEAR(static_cast<double>(built.bounds.min.x),
               static_cast<double>(imported.bounds.min.x), 1e-4);
    CHECK_NEAR(static_cast<double>(built.bounds.min.y),
               static_cast<double>(imported.bounds.min.y), 1e-4);
    CHECK_NEAR(static_cast<double>(built.bounds.min.z),
               static_cast<double>(imported.bounds.min.z), 1e-4);
    CHECK_NEAR(static_cast<double>(built.bounds.max.x),
               static_cast<double>(imported.bounds.max.x), 1e-4);
    CHECK_NEAR(static_cast<double>(built.bounds.max.y),
               static_cast<double>(imported.bounds.max.y), 1e-4);
    CHECK_NEAR(static_cast<double>(built.bounds.max.z),
               static_cast<double>(imported.bounds.max.z), 1e-4);

    // The bounding box alone would survive a mirror about the footprint's own
    // centre, so pin the asymmetric corner too: the apex at y = 9 in OSM
    // metres must land at z = -9 in world space, not +9.
    double most_negative_z = 1e18;
    for (const stratum::Vertex& v : built.vertices) {
        most_negative_z = std::min(most_negative_z, static_cast<double>(v.position.z));
    }
    CHECK_NEAR(most_negative_z, -9.0, 1e-4);
}

TEST(RuleSeed, a_seeded_area_lands_where_its_imported_mesh_does) {
    Area area;
    area.osm_id = 11;
    area.polygon = {{0.0, 0.0}, {20.0, 0.0}, {20.0, 6.0}, {10.0, 14.0}, {0.0, 6.0}};
    area.type = AreaType::Park;

    const Shape seed = seed_from_area(area);
    // Shoelace over the five points: 200, not the 220 I first wrote.
    CHECK_NEAR(geometry_area(seed.geometry), 200.0, 1e-9);

    // North in OSM metres is -z in world space, so the far corner is at -14.
    double most_negative_z = 1e18;
    for (const glm::dvec3& p : seed.geometry.positions) {
        most_negative_z = std::min(most_negative_z, seed.scope.to_world(p).z);
    }
    CHECK_NEAR(most_negative_z, -14.0, 1e-9);
}

// ============================================================================
// Attributes, proven by running a rule that reads them
// ============================================================================

TEST(RuleSeed, a_rule_reads_the_level_count_the_import_gave) {
    const Shape seed = seed_from_building(make_building());

    // The point of the whole file: the building is four storeys because OSM
    // said so, not because the rule file has a number in it.
    const GenerationResult result = run_on(seed,
        "@start\n"
        "rule M {\n"
        "    floors(attrs.get(\"osm.levels\", 3), 3.0);\n"
        "    print(\"levels\", attrs.get(\"osm.levels\", 3));\n"
        "}\n");

    CHECK_TRUE(result.ok());
    CHECK_TRUE(logged(result, "4"));
    if (!result.terminals.empty()) {
        // Four storeys of three metres.
        CHECK_NEAR(result.terminals[0].scope.size.y, 12.0, 1e-9);
    }
}

TEST(RuleSeed, a_rule_branches_on_the_building_type) {
    // One rule file, two buildings, two different results -- from the import
    // rather than from a draw. This is what makes a generated city look like
    // the city it was generated from.
    const std::string source =
        "@start\n"
        "rule M {\n"
        "    if (attrs.get(\"osm.type\", \"unknown\") == \"warehouse\") {\n"
        "        print(\"shed\");\n"
        "    } else {\n"
        "        print(\"block\");\n"
        "    }\n"
        "}\n";

    Building warehouse = make_building();
    warehouse.type = BuildingType::Warehouse;
    const GenerationResult a = run_on(seed_from_building(warehouse), source);
    CHECK_TRUE(logged(a, "shed"));
    CHECK_FALSE(logged(a, "block"));

    const GenerationResult b = run_on(seed_from_building(make_building()), source);
    CHECK_TRUE(logged(b, "block"));
    CHECK_FALSE(logged(b, "shed"));
}

TEST(RuleSeed, the_roof_words_are_the_ones_the_roof_operation_takes) {
    // OSM calls them skillion and pyramidal; roof() calls them shed and
    // pyramid. Translating here rather than in every rule file is the whole
    // reason this mapping is not area_type_name()-style passthrough.
    struct Case { RoofType type; const char* word; };
    const Case cases[] = {
        {RoofType::Flat, "flat"},        {RoofType::Gabled, "gable"},
        {RoofType::Hipped, "hip"},       {RoofType::Pyramidal, "pyramid"},
        {RoofType::Skillion, "shed"},    {RoofType::Dome, "dome"},
    };

    for (const Case& c : cases) {
        Building b = make_building();
        b.roof_type = c.type;
        const Shape seed = seed_from_building(b);
        const auto found = seed.attributes.find("osm.roof");
        CHECK_TRUE(found != seed.attributes.end());
        if (found == seed.attributes.end()) continue;
        CHECK_EQ(found->second.as_text(), std::string{c.word});

        if (std::string{c.word} == "flat") continue;
        // And every one of them is a kind roof() actually accepts, which is
        // the claim that matters: `roof(attrs.get("osm.roof", "hip"))` must
        // work with no translation table in the rule file.
        const GenerationResult result = run_on(seed,
            "@start\n"
            "rule M { extrude(6.0); select face { top : { R(); } } }\n"
            "rule R { align_scope(\"y_up\"); roof(attrs.get(\"osm.roof\", \"hip\"), 35.0); }\n");
        CHECK_TRUE(result.ok());
    }
}

TEST(RuleSeed, an_absent_name_is_an_absent_attribute) {
    // `attrs.has("osm.name")` has to be a real question. Setting an empty
    // string would make every unnamed building answer yes.
    const Shape unnamed = seed_from_building(make_building());
    CHECK_TRUE(unnamed.attributes.find("osm.name") == unnamed.attributes.end());

    Building named = make_building();
    named.name = "Custom House";
    const Shape seed = seed_from_building(named);
    CHECK_TRUE(seed.attributes.find("osm.name") != seed.attributes.end());

    const GenerationResult result = run_on(seed,
        "@start\nrule M { if (attrs.has(\"osm.name\")) { print(attrs.get(\"osm.name\", \"\")); } }\n");
    CHECK_TRUE(logged(result, "Custom House"));
}

TEST(RuleSeed, the_osm_id_survives_onto_the_generated_terminals) {
    // Attributes are inherited by children, so every piece of the generated
    // building still knows which imported feature it came from. That is what
    // makes it possible to replace one building rather than the whole city.
    const Shape seed = seed_from_building(make_building());
    const GenerationResult result = run_on(seed,
        "@start\n"
        "rule M { extrude(4.0); select face { vertical : { W(); } top : { T(); } } }\n"
        "rule W { wall_panel(); }\n"
        "rule T { wall_panel(); }\n");

    CHECK_TRUE(result.ok());
    CHECK((result.terminals.size()) >= 5);
    for (const Shape& terminal : result.terminals) {
        const auto found = terminal.attributes.find(kSeedOsmId);
        CHECK_TRUE(found != terminal.attributes.end());
        if (found != terminal.attributes.end()) {
            CHECK_NEAR(found->second.as_number(), 4242.0, 0.5);
        }
    }
}

// ============================================================================
// A whole import
// ============================================================================

TEST(RuleSeed, seeds_come_back_in_parse_order_and_one_per_building) {
    ParsedOSMData data;
    for (int i = 0; i < 5; ++i) {
        Building b = make_building();
        b.osm_id = 100 + i;
        data.buildings.push_back(b);
    }
    // A degenerate one in the middle, which must NOT be dropped: dropping it
    // renumbers every seed after it and changes what the city generates.
    data.buildings[2].footprint = {{0.0, 0.0}};

    const std::vector<Shape> seeds = seeds_from_buildings(data);
    CHECK_EQ(seeds.size(), size_t{5});
    for (size_t i = 0; i < seeds.size(); ++i) {
        const auto found = seeds[i].attributes.find(kSeedOsmId);
        CHECK_TRUE(found != seeds[i].attributes.end());
        if (found != seeds[i].attributes.end()) {
            CHECK_NEAR(found->second.as_number(), static_cast<double>(100 + i), 0.5);
        }
    }
    CHECK_TRUE(seeds[2].geometry.faces.empty());
    CHECK_FALSE(seeds[3].geometry.faces.empty());
}

TEST(RuleSeed, an_area_seed_uses_the_lower_case_word_a_rule_compares_against) {
    // area_type_name() returns "Park" for a human to read. A rule comparing
    // against "park" would never match it, and osm.type beside it is lower
    // case, so the two conventions would disagree.
    Area area;
    area.osm_id = 7;
    area.polygon = {{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}};
    area.type = AreaType::Park;

    const Shape seed = seed_from_area(area);
    const auto found = seed.attributes.find("osm.area_type");
    CHECK_TRUE(found != seed.attributes.end());
    if (found != seed.attributes.end()) {
        CHECK_EQ(found->second.as_text(), std::string{"park"});
    }

    const GenerationResult result = run_on(seed,
        "@start\nrule M { if (attrs.get(\"osm.area_type\", \"\") == \"park\") { print(\"green\"); } }\n");
    CHECK_TRUE(logged(result, "green"));
}
