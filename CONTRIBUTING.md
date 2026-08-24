# Contributing

Contributions are welcome! Here's how to get started.

## Development Setup

```bash
git clone https://github.com/YOUR_USERNAME/orbital-trajectory-simulator.git
cd orbital-trajectory-simulator
npm install
npm run dev
```

## How to Contribute

1. **Fork** the repository
2. **Create a branch**: `git checkout -b feature/your-feature-name`
3. **Make your changes** — keep physics accurate and code readable
4. **Test** the build: `npm run build`
5. **Commit**: `git commit -m "feat: describe your change"`
6. **Push**: `git push origin feature/your-feature-name`
7. **Open a Pull Request**

## Commit Convention

Use [Conventional Commits](https://www.conventionalcommits.org/):

| Prefix | Use for |
|---|---|
| `feat:` | New feature |
| `fix:` | Bug fix |
| `physics:` | Physics model change or correction |
| `docs:` | Documentation only |
| `style:` | Formatting, no logic change |
| `refactor:` | Code restructure, no behaviour change |
| `perf:` | Performance improvement |

## Physics Changes

If you modify the physics model:
- Cite your sources (paper, textbook, or standard)
- Update the README physics section
- Update the in-app README tab (`App.jsx`)
- Validate against known orbital parameters (e.g. ISS at 400 km ≈ 7.67 km/s)

## Areas for Contribution

- 3D trajectory simulation
- Thrust / delta-v burn phases
- J₂ oblateness perturbation
- Multiple body tracking
- TLE import / export
- Unit tests for the physics engine
