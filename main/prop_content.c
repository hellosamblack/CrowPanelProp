/* prop_content — author-editable archive content. See prop_content.h.
 *
 * PLACEHOLDER CONTENT: real-Earth desert stand-ins for the Traxian canon. The book
 * author replaces the strings below; the ARCHIVE UI needs no changes when they do.
 * Keep bodies a few short sentences — they wrap into a 7" panel read on camera.
 */
#include "prop_content.h"

/* --- TRAXIAN CLIMATE ------------------------------------------------------- */
static const prop_entry_t climate_entries[] = {
    { "THERMAL PROFILE",
      "Surface temperatures swing violently between day and night. Midday highs "
      "reach 48 C in open dune fields; clear-sky nights fall below 4 C. The thin, "
      "dry air holds little heat once the sun drops - exposure after dark is the "
      "primary survival hazard." },
    { "SEASONS",
      "Two seasons dominate: a long dry season of relentless sun and a brief wet "
      "interval of violent, short-lived storms. Annual rainfall averages under "
      "90 mm, most of it arriving in a handful of flash floods." },
    { "WIND & DUST",
      "Prevailing winds drive standing dust storms that can blot the horizon for "
      "days. Fine grit infiltrates every seam and bearing; sealed housings and "
      "regular intake purges are mandatory for field equipment." },
    { "WATER TABLE",
      "Permanent water sits deep, surfacing only at scattered springs along the "
      "fault lines. Morning dew condensing on cool rock is a measurable, if meagre, "
      "source exploited by both wildlife and travellers." },
};

/* --- A MAP OF THE DESERT (regions as entries until a map image is added) --- */
static const prop_entry_t map_entries[] = {
    { "THE GREAT ERG",
      "A vast sea of wind-built dunes, some cresting 200 m. Trackless and shifting; "
      "navigation relies on celestial fixes and buried marker cairns. No permanent "
      "water across its interior." },
    { "THE SALT FLATS",
      "A blinding white pan, dead flat to the horizon. Hard-packed and fast to "
      "cross by day, but a thin brine crust hides soft mud after rare rains. Mirage "
      "distortion makes range estimates unreliable." },
    { "THE RIFT ESCARPMENT",
      "A long fault scarp of banded rock marking the desert's eastern edge. Springs "
      "seep along its base, supporting the only reliable green corridor. Caves in "
      "the cliff face offer shelter from storms." },
    { "THE STONE BARRENS",
      "A wind-scoured plain of dark gravel and shattered rock - hamada. Brutal on "
      "foot and gear, but firm underfoot and free of dunes. Daytime heat radiating "
      "from the stone is extreme.",
    },
};

/* --- TRAXIAN WILDLIFE BIOS ------------------------------------------------- */
static const prop_entry_t wildlife_entries[] = {
    { "FENNEC FOX",
      "A small nocturnal canid with oversized ears that shed heat and locate prey "
      "underground. Dens by day to escape the surface heat; needs almost no free "
      "water, drawing moisture from its food. Wary but not aggressive." },
    { "SIDEWINDER",
      "A venomous pit viper that moves by throwing loops of its body sideways across "
      "loose sand, leaving distinctive J-shaped tracks. Ambush hunter; buries itself "
      "to wait. Most active at dusk and after dark." },
    { "DROMEDARY",
      "Large herbivore tolerant of extreme dehydration, able to lose a third of its "
      "body water and recover. Stores fat - not water - in a single dorsal hump. "
      "Domesticated stock are the backbone of desert travel." },
    { "DARKLING BEETLE",
      "A palm-sized scavenger that harvests fog by standing head-down on dune crests, "
      "letting condensed droplets run to its mouth. Active in the cool early morning; "
      "burrows during the heat of day." },
};

/* --- PLANT GUIDE (the "Wikipedia download") ------------------------------- */
static const prop_entry_t plant_entries[] = {
    { "SAGUARO",
      "A giant columnar cactus that stores tons of water in pleated, expandable "
      "tissue. Shallow roots spread wide to capture rare rainfall fast. Grows "
      "slowly over centuries; its night-opening flowers feed bats and insects." },
    { "CREOSOTE BUSH",
      "A resinous evergreen shrub that survives on almost no water and poisons the "
      "soil around it to suppress competitors. Among the longest-lived organisms "
      "known - some clonal rings are thousands of years old." },
    { "AGAVE",
      "A rosette of thick, water-storing leaves edged with spines. Lives years as a "
      "low rosette, then sends up a single towering flower stalk and dies. Sap and "
      "fibre are both harvested by desert peoples." },
    { "DATE PALM",
      "A tall palm anchoring oasis agriculture: deep roots reach buried water while "
      "the crown tolerates fierce sun. A mature tree yields heavy clusters of "
      "sugar-rich fruit, a keystone trade staple." },
};

const prop_section_t prop_sections[] = {
    { "CLIMATE",  climate_entries,  (int)(sizeof(climate_entries)  / sizeof(climate_entries[0]))  },
    { "MAP",      map_entries,      (int)(sizeof(map_entries)      / sizeof(map_entries[0]))      },
    { "WILDLIFE", wildlife_entries, (int)(sizeof(wildlife_entries) / sizeof(wildlife_entries[0])) },
    { "PLANTS",   plant_entries,    (int)(sizeof(plant_entries)    / sizeof(plant_entries[0]))    },
};
const int prop_section_count = (int)(sizeof(prop_sections) / sizeof(prop_sections[0]));

const char *const prop_date_stamp = "TRAXIAN STD  4471.212";
