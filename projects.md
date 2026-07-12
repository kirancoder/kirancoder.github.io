---
layout: single
title: "Research & Projects"
---

<section class="research-intro">
  <div class="research-header">
    <h1>Research & Projects</h1>
    <p class="research-bio">
      My research focuses on the intersection of <strong>Optimal Control Theory</strong>, <strong>Numerical Optimization</strong>, and <strong>Machine Learning</strong> to develop robust, high-precision guidance and control laws for autonomous aerospace vehicles.
    </p>
  </div>

  <div class="research-meta">
    <div class="meta-block">
      <h3>Research Interests</h3>
      <div class="tag-cloud">
        <span class="research-tag">Trajectory Optimization</span>
        <span class="research-tag">Guidance Navigation & Control</span>
        <span class="research-tag">Planetary Landing</span>
        <span class="research-tag">PINNs</span>
        <span class="research-tag">Theory of Functional Connections</span>
        <span class="research-tag">Sequential Convex Optimization</span>
        <span class="research-tag">Numerical Optimization</span>
      </div>
    </div>

    <div class="meta-block">
      <h3>Current Research</h3>
      <p class="research-focus">
        Currently investigating the use of <strong>Physics-Informed Neural Networks (PINNs)</strong> and <strong>Deep Learning</strong> to solve the Hamilton-Jacobi-Bellman equations, aiming to replace traditional computationally expensive trajectory optimization with real-time, learnable control policies for spacecraft re-entry and landing.
      </p>
    </div>
  </div>
</section>

<section class="projects-section">
  <h2 class="section-title">Featured Research</h2>
  
  <div class="project-grid">
    <!-- Project 1: Trajectory Optimization using ML -->
    <article class="project-card featured">
      <div class="card-visual">
        <svg viewBox="0 0 400 200" class="project-svg">
          <path d="M20,180 Q100,150 150,100 T380,20" stroke="var(--accent-primary)" stroke-width="3" fill="none" />
          <circle cx="20" cy="180" r="4" fill="white" />
          <circle cx="380" cy="20" r="4" fill="white" />
          <path d="M150,100 L160,110" stroke="var(--text-muted)" stroke-width="1" />
          <text x="165" y="115" fill="var(--text-muted)" font-size="12">Optimized Path</text>
        </svg>
      </div>
      <div class="card-content">
        <div class="card-header">
          <span class="category-tag">Trajectory Optimization • ML</span>
          <h3>Trajectory Optimization using Machine Learning</h3>
          <figure style="margin: 1rem 0; text-align: center;">
            <svg viewBox="0 0 400 120" width="100%" height="120" xmlns="http://www.w3.org/2000/svg">
              <rect x="20" y="40" width="80" height="40" rx="5" fill="none" stroke="var(--accent-primary)" stroke-width="2" />
              <text x="60" y="65" text-anchor="middle" fill="var(--text-main)" font-size="12">Boundary Cond.</text>
              
              <line x1="100" y1="60" x2="140" y2="60" stroke="var(--text-muted)" stroke-width="2" marker-end="url(#arrowhead)" />
              
              <rect x="140" y="30" width="120" height="60" rx="5" fill="none" stroke="var(--accent-primary)" stroke-width="2" />
              <text x="200" y="65" text-anchor="middle" fill="var(--text-main)" font-size="12">PINN Model</text>
              
              <line x1="260" y1="60" x2="300" y2="60" stroke="var(--text-muted)" stroke-width="2" marker-end="url(#arrowhead)" />
              
              <rect x="300" y="40" width="80" height="40" rx="5" fill="none" stroke="var(--accent-primary)" stroke-width="2" />
              <text x="340" y="65" text-anchor="middle" fill="var(--text-main)" font-size="12">Opt. Traj.</text>
              
              <defs>
                <marker id="arrowhead" markerWidth="10" markerHeight="7" refX="0" refY="3.5" orient="auto">
                  <polygon points="0 0, 10 3.5, 0 7" fill="var(--text-muted)" />
                </marker>
              </defs>
            </svg>
            <figcaption style="font-size: 0.85rem; color: var(--text-muted); margin-top: 0.5rem;">
              Figure: Trajectory Optimization Architecture
            </figcaption>
          </figure>
          <p class="summary">This work develops a mesh-free framework for solving coupled state-costate equations using Physics-Informed Neural Networks (PINNs).</p>
        </div>
        
        <div class="card-story">
          <figure style="margin: 1rem 0; text-align: center;">
            <svg viewBox="0 0 200 250" width="200" height="250" xmlns="http://www.w3.org/2000/svg">
              <!-- Box 1 -->
              <rect x="25" y="10" width="150" height="40" rx="5" fill="none" stroke="var(--accent-primary)" stroke-width="2" />
              <text x="100" y="35" text-anchor="middle" fill="var(--text-main)" font-size="12">Initial State</text>
              
              <!-- Arrow 1 -->
              <line x1="100" y1="50" x2="100" y2="70" stroke="var(--text-muted)" stroke-width="2" marker-end="url(#arrowhead)" />
              
              <!-- Box 2 -->
              <rect x="25" y="70" width="150" height="40" rx="5" fill="none" stroke="var(--accent-primary)" stroke-width="2" />
              <text x="100" y="95" text-anchor="middle" fill="var(--text-main)" font-size="12">Powered Descent</text>
              
              <!-- Arrow 2 -->
              <line x1="100" y1="110" x2="100" y2="130" stroke="var(--text-muted)" stroke-width="2" marker-end="url(#arrowhead)" />
              
              <!-- Box 3 -->
              <rect x="25" y="130" width="150" height="40" rx="5" fill="none" stroke="var(--accent-primary)" stroke-width="2" />
              <text x="100" y="155" text-anchor="middle" fill="var(--text-main)" font-size="12">Terminal Guidance</text>
              
              <!-- Arrow 3 -->
              <line x1="100" y1="170" x2="100" y2="190" stroke="var(--text-muted)" stroke-width="2" marker-end="url(#arrowhead)" />
              
              <!-- Box 4 -->
              <rect x="25" y="190" width="150" height="40" rx="5" fill="none" stroke="var(--accent-primary)" stroke-width="2" />
              <text x="100" y="215" text-anchor="middle" fill="var(--text-main)" font-size="12">Landing</text>
              
              <defs>
                <marker id="arrowhead" markerWidth="10" markerHeight="7" refX="0" refY="3.5" orient="auto">
                  <polygon points="0 0, 10 3.5, 0 7" fill="var(--text-muted)" />
                </marker>
              </defs>
            </svg>
            <figcaption style="font-size: 0.85rem; color: var(--text-muted); margin-top: 0.5rem;">
              Figure: Mission Profile Sequence
            </figcaption>
          </figure>
          <div class="story-item">
            <strong>Problem:</strong> Indirect methods for trajectory optimization exhibit high sensitivity to initial costate guesses, which frequently leads to convergence failures.
          </div>
          <div class="story-item">
            <strong>Method:</strong> Physics-Informed Neural Networks (PINNs) are employed to learn the mapping from boundary conditions to optimal trajectories.
          </div>
          <div class="story-item">
            <strong>Implementation:</strong> TODO
          </div>
          <div class="story-item">
            <strong>Results:</strong> The proposed method generated over 100,000 high-accuracy trajectory pairs, significantly reducing the search space for traditional numerical solvers.
          </div>
          <figure style="margin: 1rem 0; text-align: center;">
            <svg viewBox="0 0 400 200" width="100%" height="200" xmlns="http://www.w3.org/2000/svg">
              <!-- Axes -->
              <line x1="40" y1="160" x2="360" y2="160" stroke="var(--text-muted)" stroke-width="2" />
              <line x1="40" y1="160" x2="40" y2="40" stroke="var(--text-muted)" stroke-width="2" />
              
              <!-- Labels -->
              <text x="200" y="180" text-anchor="middle" fill="var(--text-muted)" font-size="12">Epochs / Iterations</text>
              <text x="20" y="100" text-anchor="middle" fill="var(--text-muted)" font-size="12" transform="rotate(-90, 20, 100)">Loss / Error</text>
              
              <!-- Convergence Curve (Exponential Decay) -->
              <path d="M 40 50 Q 100 140 360 150" stroke="var(--accent-primary)" stroke-width="3" fill="none" />
              <path d="M 40 60 Q 120 150 360 155" stroke="var(--accent-secondary)" stroke-width="2" fill="none" stroke-dasharray="4" />
              
              <!-- Legend -->
              <line x1="280" y1="60" x2="300" y2="60" stroke="var(--accent-primary)" stroke-width="3" />
              <text x="310" y="65" fill="var(--text-main)" font-size="11">PINN</text>
              
              <line x1="280" y1="80" x2="300" y2="80" stroke="var(--accent-secondary)" stroke-width="2" stroke-dasharray="4" />
              <text x="310" y="85" fill="var(--text-main)" font-size="11">Baseline</text>
            </svg>
            <figcaption style="font-size: 0.85rem; color: var(--text-muted); margin-top: 0.5rem;">
              Figure: Optimization Convergence
            </figcaption>
          </figure>
          <div class="story-item">
            <strong>Future Work:</strong> TODO
          </div>
        </div>
        
        <div class="card-footer">
          <div class="tech-pills">
            <span>Python</span> <span>PyTorch</span> <span>IPOPT</span>
          </div>
          <div class="card-actions">
            <a href="javascript:void(0)" class="action-btn" title="Link pending">GitHub</a>
            <a href="javascript:void(0)" class="action-btn outline" title="Link pending">Paper</a>
          </div>
        </div>
      </div>
    </article>

    <!-- Project 2: TFC -->
    <article class="project-card featured">
      <div class="card-visual">
        <svg viewBox="0 0 400 200" class="project-svg">
          <path d="M50,150 C100,150 150,50 200,50 S300,150 350,150" stroke="var(--accent-primary)" stroke-width="2" fill="none" />
          <path d="M50,150 L350,150" stroke="var(--text-muted)" stroke-width="1" stroke-dasharray="4" />
          <circle cx="200" cy="50" r="3" fill="white" />
          <text x="210" y="45" fill="var(--text-muted)" font-size="12">Analytical Approximation</text>
        </svg>
      </div>
      <div class="card-content">
        <div class="card-header">
          <span class="category-tag">Numerical Methods • TFC</span>
          <h3>Theory of Functional Connections (TFC)</h3>
          <p class="summary">Application of TFC to solve boundary value problems without iterative grids.</p>
        </div>
        
        <div class="card-story">
          <div class="story-item">
            <strong>Problem:</strong> Traditional numerical methods require dense grids and iterative solvers, which are computationally expensive for high-dimensional spaces.
          </div>
          <div class="story-item">
            <strong>Method:</strong> Utilized TFC to represent the solution as a constrained functional, converting the ODE into a linear system of algebraic equations.
          </div>
          <div class="story-item">
            <strong>Implementation:</strong> TODO
          </div>
          <div class="story-item">
            <strong>Results:</strong> Achieved spectral accuracy with significantly fewer degrees of freedom compared to finite-difference methods.
          </div>
          <div class="story-item">
            <strong>Future Work:</strong> TODO
          </div>
        </div>
        
        <div class="card-footer">
          <div class="tech-pills">
            <span>MATLAB</span> <span>Numerical Analysis</span>
          </div>
          <div class="card-actions">
            <a href="javascript:void(0)" class="action-btn" title="Link pending">GitHub</a>
          </div>
        </div>
      </div>
    </article>

    <!-- Project 3: Planetary Landing -->
    <article class="project-card featured">
      <div class="card-visual">
        <svg viewBox="0 0 400 200" class="project-svg">
          <path d="M350,50 Q250,100 200,180" stroke="var(--accent-primary)" stroke-width="3" fill="none" />
          <path d="M100,180 L300,180" stroke="white" stroke-width="4" />
          <circle cx="200" cy="180" r="5" fill="var(--accent-primary)" />
          <text x="210" y="170" fill="var(--text-muted)" font-size="12">Targeted Landing</text>
        </svg>
      </div>
      <div class="card-content">
        <div class="card-header">
          <span class="category-tag">Planetary Landing • GNC</span>
          <h3>Optimal Lunar Descent Trajectories</h3>
          <p class="summary">Designing fuel-optimal powered descent guidance for lunar landing missions.</p>
        </div>
        
        <div class="card-story">
          <div class="story-item">
            <strong>Problem:</strong> Balancing fuel efficiency with strict safety constraints on velocity and glide-slope angles.
          </div>
          <div class="story-item">
            <strong>Method:</strong> Developed a Sequential Convex Programming (SCP) based guidance law to solve the non-convex optimal control problem in real-time.
          </div>
          <div class="story-item">
            <strong>Result:</strong> Demonstrated robust convergence to optimal landing profiles with guaranteed constraint satisfaction.
          </div>
        </div>
        
        <div class="card-footer">
          <div class="tech-pills">
            <span>Python</span> <span>CVXPY</span> <span>SCP</span>
          </div>
          <div class="card-actions">
            <a href="javascript:void(0)" class="action-btn" title="Link pending">GitHub</a>
            <a href="javascript:void(0)" class="action-btn outline" title="Link pending">Report</a>
          </div>
        </div>
      </div>
    </article>

    <!-- Project 4: GNC Aircraft -->
    <article class="project-card featured">
      <div class="card-visual">
        <svg viewBox="0 0 400 200" class="project-svg">
          <path d="M100,100 L300,100 M200,50 L200,150 M150,75 L250,125 M150,125 L250,75" stroke="var(--accent-primary)" stroke-width="2" fill="none" />
          <circle cx="200" cy="100" r="4" fill="white" />
          <text x="210" y="90" fill="var(--text-muted)" font-size="12">6-DOF Dynamics</text>
        </svg>
      </div>
      <div class="card-content">
        <div class="card-header">
          <span class="category-tag">Aerospace Systems • GNC</span>
          <h3>GNC-Oriented 6-DOF Aircraft Model</h3>
          <p class="summary">Development of a high-fidelity nonlinear aircraft dynamics model for the validation of advanced guidance, navigation, and control laws.</p>
        </div>
        
        <div class="card-story">
          <div class="story-item">
            <strong>Problem:</strong> Conventional linear approximations often fail to capture critical cross-coupling effects and nonlinearities inherent in high-angle-of-attack flight regimes, compromising the fidelity of control law validation.
          </div>
          <div class="story-item">
            <strong>Method:</strong> A six-degree-of-freedom (6-DOF) nonlinear rigid-body dynamics model was developed, incorporating full equations of motion and detailed aerodynamic coefficient mappings.
          </div>
          <div class="story-item">
            <strong>Implementation:</strong> The model was implemented in MATLAB/Simulink, featuring modular subsystems for atmospheric properties, actuator dynamics, and numerical state integration.
          </div>
          <div class="story-item">
            <strong>Results:</strong> The model's fidelity was verified through the identification and validation of characteristic longitudinal (Phugoid) and lateral-directional (Dutch Roll) modes across a broad flight envelope.
          </div>
          <div class="story-item">
            <strong>Future Work:</strong> TODO
          </div>
        </div>
        
        <div class="card-footer">
          <div class="tech-pills">
            <span>MATLAB</span> <span>Simulink</span> <span>Control Systems</span>
          </div>
          <div class="card-actions">
            <a href="javascript:void(0)" class="action-btn" title="Link pending">GitHub</a>
          </div>
        </div>
      </div>
    </article>
  </div>

  <h2 class="section-title secondary">Other Projects</h2>
  <div class="project-grid">
    <article class="project-card">
      <div class="card-content">
        <div class="card-header">
          <span class="category-tag">Conceptual Design</span>
          <h3>UAV Conceptual Design</h3>
          <p class="summary">Design of a portable, constraint-driven fixed-wing UAV.</p>
        </div>
        <div class="card-story">
          <div class="story-item">
            <strong>Problem:</strong> Strictly constrained physical dimensions (9 × 30 × 31 cm) for portability.
          </div>
          <div class="story-item">
            <strong>Method:</strong> Used a hybrid structure of balsa and 3D-printed components with aerodynamic sizing.
          </div>
          <div class="story-item">
            <strong>Result:</strong> Developed a modular, lightweight prototype meeting all sizing constraints.
          </div>
        </div>
        <div class="card-footer">
          <div class="tech-pills">
            <span>SolidWorks</span> <span>Balsa/Carbon Fiber</span>
          </div>
          <div class="card-actions">
            <a href="javascript:void(0)" class="action-btn" title="Link pending">Details</a>
          </div>
        </div>
      </div>
    </article>
  </div>
</section>

<section class="skills-section">
  <h2 class="section-title">Technical Competencies</h2>
  <div class="skills-grid">
    <!-- Card 1: Programming -->
    <div class="skill-card">
      <div class="skill-header">
        <div class="skill-icon-svg icon-programming"></div>
        <h3>Programming</h3>
      </div>
      <div class="skill-list">
        <span>Python</span> <span>MATLAB</span> <span>C++</span> <span>Git</span>
      </div>
    </div>

    <!-- Card 2: Scientific Computing -->
    <div class="skill-card">
      <div class="skill-header">
        <div class="skill-icon-svg icon-science"></div>
        <h3>Scientific Computing</h3>
      </div>
      <div class="skill-list">
        <span>NumPy</span> <span>SciPy</span> <span>Pandas</span> <span>Matplotlib</span>
      </div>
    </div>

    <!-- Card 3: Machine Learning -->
    <div class="skill-card">
      <div class="skill-header">
        <div class="skill-icon-svg icon-ml"></div>
        <h3>Machine Learning</h3>
      </div>
      <div class="skill-list">
        <span>PyTorch</span> <span>JAX</span> <span>Physics-Informed Neural Networks</span>
      </div>
    </div>

    <!-- Card 4: Optimization & Optimal Control -->
    <div class="skill-card">
      <div class="skill-header">
        <div class="skill-icon-svg icon-opt"></div>
        <h3>Optimization & Optimal Control</h3>
      </div>
      <div class="skill-list">
        <span>IPOPT</span> <span>PyGMO</span> <span>Sequential Convex Optimization</span> <span>Direct Methods</span> <span>Multiple Shooting</span> <span>Hermite-Simpson</span> <span>Trapezoidal Collocation</span>
      </div>
    </div>

    <!-- Card 5: Aerospace Engineering -->
    <div class="skill-card">
      <div class="skill-header">
        <div class="skill-icon-svg icon-aero"></div>
        <h3>Aerospace Engineering</h3>
      </div>
      <div class="skill-list">
        <span>Trajectory Optimization</span> <span>Guidance, Navigation & Control</span> <span>Planetary Landing</span> <span>Orbital Mechanics</span> <span>Flight Dynamics</span> <span>6-DOF Simulation</span>
      </div>
    </div>

    <!-- Card 6: Research & Development -->
    <div class="skill-card">
      <div class="skill-header">
        <div class="skill-icon-svg icon-rd"></div>
        <h3>Research & Development</h3>
      </div>
      <div class="skill-list">
        <span>Theory of Functional Connections (TFC)</span> <span>LaTeX</span> <span>Jupyter</span> <span>Linux</span>
      </div>
    </div>
  </div>
</section>
