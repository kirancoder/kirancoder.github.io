# Website Modernization & Audit Plan

This document outlines the comprehensive audit of the `kirancoder.github.io` repository and provides a prioritized roadmap for transforming the site into a premium, modern professional portfolio.

## 1. Executive Summary
The current website is a hybrid of a Jekyll-based template (`minimal-mistakes`) and custom manual overrides. While it contains high-quality technical content (Aerospace/Optimal Control), the visual presentation is inconsistent, transitioning abruptly between a legacy white-theme (in `style.css`) and a modern deep-space dark-theme (in `main.scss`). 

**Core Verdict:** The site has a strong technical foundation but suffers from "CSS schizophrenia" and incomplete implementation of the new design system.

---

## 2. Audit Findings

### 2.1 Duplicate & Conflicting CSS
- **Finding:** There are two primary stylesheets: `/assets/style.css` (Light theme, legacy) and `/assets/main.scss` (Dark theme, modern).
- **Issue:** `_includes/head.html` loads `main.css` (compiled from `main.scss`). However, `style.css` exists in the repo and contains conflicting definitions for `.wrapper`, `body`, and `.site-title`.
- **Risk:** Maintenance overhead and unpredictable rendering if both are ever loaded.

### 2.2 Dead Code & Unused Assets
- **Finding:** `assets/style.css` is likely redundant as the site has shifted to a dark theme.
- **Finding:** `_layouts/note.html` exists but its usage is minimal compared to `single`.
- **Finding:** The `_site` directory is committed to the repository, which is a major anti-pattern for Jekyll sites.

### 2.3 Accessibility (a11y)
- **Issues:**
    - **Contrast:** Some `var(--text-muted)` colors on `var(--bg-deep)` may fail WCAG AA contrast ratios.
    - **Semantic HTML:** The `index.md` uses `div` for tech-stack badges instead of a list (`ul`/`li`).
    - **ARIA:** Navigation dropdowns lack `aria-expanded` and `aria-haspopup` attributes.
    - **Images/SVGs:** Several SVGs in `projects.md` lack `<title>` or `aria-label` for screen readers.

### 2.4 Responsive Design
- **Issues:**
    - **Project Grid:** The project cards in `projects.md` are complex. While the CSS uses `grid-template-columns: repeat(auto-fit, ...)`, the internal SVGs may overflow on very small screens (<360px).
    - **Header:** The custom header is fixed; on mobile, the dropdown menu might obstruct significant content if not handled with a proper hamburger menu.

### 2.5 SEO & Metadata
- **Issues:**
    - **Missing Meta:** `_config.yml` has a placeholder email and LinkedIn URL.
    - **Page Titles:** `about.md` and `research.md` have basic titles; they lack descriptive meta-descriptions for search engines.
    - **OpenGraph:** No explicit OpenGraph images defined for social sharing.

### 2.6 Broken Links & Content Gaps
- **Broken Links:** 
    - `projects.md` has multiple `href="#"` for GitHub and Paper links.
    - `index.md` links to `/assets/pdf/Kiran_Aralikatti_CV.pdf` which is not present in the listed directory structure.
- **Content Gaps:** 
    - `projects.md` contains several "TODO" markers in the "Implementation" and "Future Work" sections.
    - `research.md` has an empty "Research Methodology" section.

### 2.7 Inconsistent Styling
- **Finding:** The `index.md` uses a high-end "Hero" design, but `about.md`, `research.md`, and `cv.md` revert to a very basic layout.
- **Finding:** The "Research & Projects" page introduces new CSS classes (e.g., `.research-intro`, `.project-card`) that are **not defined** in `main.scss`.

---

## 3. Page-by-Page Recommendations

### Home (`index.md`)
- **Change:** Convert tech-stack badges to a semantic list.
- **Why:** Accessibility and SEO.
- **Impact:** Low | **Difficulty:** Easy.

### About (`about.md`)
- **Change:** Implement a "Bio" layout with a professional headshot and a structured "Experience" timeline.
- **Why:** Currently too sparse; looks like a placeholder.
- **Impact:** High | **Difficulty:** Medium.

### Research & Projects (`projects.md`)
- **Change:** Move all project-specific CSS into `main.scss`.
- **Why:** The styles are missing from the main CSS file, causing layout collapse.
- **Impact:** Critical | **Difficulty:** Medium.
- **Change:** Expand every research project to include: **Problem, Background, Mathematical Model, Method, Implementation, Results, and Future Work**.
- **Why:** Academic rigor and professionalism.
- **Impact:** Critical | **Difficulty:** High.
- **Change:** Add required technical figures for major projects: **SVG diagrams, Trajectory figures, Convergence plots, State/Control history, and Architecture diagrams**.
- **Why:** Empirical validation of research.
- **Impact:** Critical | **Difficulty:** High.

### Research Page (`research.md`)
- **Change:** Transform into a **laboratory-style research overview**.
- **Why:** Better alignment with aerospace research standards.
- **Impact:** High | **Difficulty:** Medium.

### CV (`cv.md`)
- **Change:** Transform from a simple list to a **web-native CV layout**.
- **Why:** Improved readability and professional presentation.
- **Impact:** High | **Difficulty:** Medium.

### Technical Notes (`_notes/`)
- **Change:** Refactor notes into **publication-style articles**.
- **Why:** Increases authority and readability.
- **Impact:** Medium | **Difficulty:** Medium.

### Contact (`contact.md`)
- **Change:** Improve layout and add a professional footer.
- **Why:** Consistency and UX.
- **Impact:** Low | **Difficulty:** Easy.

---

## 4. Modernization Roadmap

### Phase 1: Technical Stabilization (Immediate)
*Goal: Fix broken styles and remove clutter.*
1. **CSS Consolidation:** Verify unused CSS files, then consolidate styles and move missing `projects.md` styles into `main.scss`.
2. **Repo Cleanup:** Remove tracked build artifacts (e.g., `_site` folder).
3. **Link Audit:** Fix all `href="#"` and verify PDF paths.
4. **Semantic Fixes:** Implement semantic HTML for tech-stack and navigation.

### Phase 2: Visual Identity & Consistency (Short Term)
*Goal: Achieve a unified, premium aesthetic across all pages.*
1. **Unified Layouts:** Create custom layouts for "Single" pages that match the Hero aesthetic (Glassmorphism, consistent spacing).
2. **Dark Mode Polish:** Refine the color palette for better contrast and depth (WCAG AA compliance).
3. **Responsive Polish:** Optimize SVGs for mobile and implement a mobile-first navigation drawer.

### Phase 3: Academic & Technical Expansion (Medium Term)
*Goal: Transform the site into a high-authority research portfolio.*
1. **Project Deep-Dive:** Rewrite research projects with the mandatory 7-point structure (Problem $	o$ Future Work).
2. **Technical Visualization:** Integrate all required trajectory and convergence figures into project pages.
3. **Research Overview:** Implement the laboratory-style overview on the Research page.
4. **Web-Native CV:** Build the interactive CV layout.
5. **Note Refactoring:** Convert technical notes into publication-style articles.

### Phase 4: SEO & Final Polish (Long Term)
*Goal: Maximize discoverability and professional impact.*
1. **SEO Optimization:** Fill out `_config.yml` (email, social links) and add page-specific meta-descriptions.
2. **a11y Pass:** Final audit for ARIA labels and keyboard navigation.
3. **Metadata:** Configure OpenGraph tags for professional social sharing.
