# Phase 1 Review: Website Modernization

This document provides a technical audit of the `kirancoder.github.io` repository after the completion of Phase 1 (Style Consolidation & Cleanup).

## 1. CSS Analysis

### 1.1 Duplicated & Conflicting CSS
- **Theme Conflicts:** The `main.scss` file relies heavily on `!important` for `.custom-header`, `.site-title`, and `.nav-link`. This indicates a conflict with the underlying Jekyll theme (`minimal-mistakes`). While functional, it is a brittle approach.
- **Internal Duplication:** No significant duplication found within `main.scss` itself.

### 1.2 Unused Selectors
- **`.cta-button.secondary`**: Defined in `main.scss` but not utilized in `index.md` or `projects.md`.
- **`.site-footer`**: The class is used in `footer.html`, but there are no corresponding styles in `main.scss` to handle the container's layout, relying instead on theme defaults and inline styles.

## 2. Responsive Design Issues

### 2.1 Navigation (Critical)
- **Missing Mobile Menu:** The header uses a fixed layout with a horizontal `nav-menu`. There is no "hamburger" menu or mobile-specific navigation toggle. On small screens, the navigation items will either overflow the viewport or wrap poorly, obstructing the page content.
- **Fixed Header Offset:** `.page-content` has a hardcoded `padding-top: 80px`. If the header height changes (e.g., due to wrapped text on mobile), content may be overlapped or have excessive whitespace.

### 2.2 Component Scaling
- **Project SVGs:** While `viewBox` is used, the inline SVGs in `projects.md` may become too small to be legible on very narrow devices (<360px).

## 3. Accessibility (a11y) Issues

### 3.1 Semantic HTML & ARIA
- **Dropdown Menu:** The `.dropdown` button in `header.html` lacks critical ARIA attributes:
    - Missing `aria-haspopup="true"`
    - Missing `aria-expanded="false/true"`
- **SVG Accessibility:** Most SVGs in `projects.md` lack `<title>` tags or `aria-label` attributes, making them invisible or meaningless to screen reader users.
- **CTA Labels:** The "Download CV" link uses a `title` attribute for its pending status, but an `aria-label` would be more robust for assistive technologies.

### 3.2 Color Contrast
- **Muted Text:** `var(--text-muted)` (#64748b) on `var(--bg-deep)` (#0a0a0a) may fall below the WCAG AA contrast ratio (4.5:1) for small text, potentially impacting readability for visually impaired users.

## 4. Broken & Suboptimal Layouts

### 4.1 Footer Implementation
- **Inline Styles:** The `footer.html` file contains a large block of inline CSS (`style="text-align: center; color: #777; ..."`). This violates the separation of concerns and makes global theme updates difficult.
- **Styling Gap:** There is no global `.site-footer` class definition in `main.scss` to manage the footer's structural layout.

### 4.2 Content Gaps
- **TODOs:** Several "TODO" markers remain in `projects.md` (Implementation and Future Work sections), which should be addressed before a production release.

## Summary Scorecard

| Category | Score | Status |
| :--- | :---: | :--- |
| **CSS Architecture** | 7/10 | Consolidated, but uses `!important` hacks. |
| **Responsiveness** | 4/10 | Hero/Grids are good; Navigation is broken on mobile. |
| **Accessibility** | 5/10 | Semantic structure improved; ARIA and SVGs missing. |
| **Layout Integrity** | 6/10 | Core pages work; Footer is poorly implemented. |
