#ifndef _PROP_CONTENT_H_
#define _PROP_CONTENT_H_

/* prop_content — the in-world data archive (Zarrah's "multi-data interface").
 *
 * This is the *content layer* the ARCHIVE browser renders: a flat, author-editable
 * set of sections, each holding a list of entries (title + body). It owns no UI and
 * no logic — prop_ui.c reads these tables and lays them out with the style kit.
 *
 * AUTHORING: everything here is placeholder, meant to be replaced by the book
 * author. The stand-ins are deliberately *real Earth* desert flora/fauna/climate so
 * the prop reads as a believable field archive until the Traxian canon drops in.
 * To add/edit content, just edit the tables in prop_content.c — no UI changes
 * needed. To add an image to a section later, see the LVGL image pipeline in
 * CLAUDE.md (PNG -> C array, PSRAM-resident) and extend
 * prop_entry_t with an image source.
 */

/* One archive entry: a headline + a body paragraph (LVGL wraps the body). */
typedef struct {
    const char *title;
    const char *body;
} prop_entry_t;

/* One archive section: a tab in the ARCHIVE browser, holding a list of entries. */
typedef struct {
    const char *name;            /* tab label, e.g. "PLANT GUIDE" */
    const prop_entry_t *entries;
    int count;
} prop_section_t;

/* The archive: sections in tab order. Defined in prop_content.c. */
extern const prop_section_t prop_sections[];
extern const int prop_section_count;

/* In-world date stamp for the console status strip (no RTC/SNTP yet — the clock
 * shows uptime). Author-editable placeholder. */
extern const char *const prop_date_stamp;

#endif /* _PROP_CONTENT_H_ */
