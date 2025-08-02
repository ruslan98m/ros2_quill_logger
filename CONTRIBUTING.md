# Contributing to Quill Logger for ROS2

Thank you for your interest in contributing to Quill Logger for ROS2! This document provides guidelines and information for contributors.

## Table of Contents

- [Code of Conduct](#code-of-conduct)
- [Getting Started](#getting-started)
- [Development Setup](#development-setup)
- [Code Style](#code-style)
- [Testing](#testing)
- [Documentation](#documentation)
- [Pull Request Process](#pull-request-process)
- [Release Process](#release-process)

## Code of Conduct

This project and everyone participating in it is governed by our Code of Conduct. By participating, you are expected to uphold this code.

## Getting Started

1. Fork the repository
2. Clone your fork locally
3. Create a feature branch
4. Make your changes
5. Test your changes
6. Submit a pull request

## Development Setup

### Prerequisites

- ROS2 Humble or later
- CMake 3.8 or later
- C++17 compatible compiler
- Quill library installed

### Setup Development Environment

```bash
# Clone your fork
git clone https://github.com/your-username/quill_logger.git
cd quill_logger

# Install dependencies
sudo apt-get install clang-format doxygen

# Build the project
make build
```

## Code Style

We use the following tools to maintain code quality:

### Clang Format

We use Google C++ Style Guide with some modifications. Run formatting:

```bash
make format
```

Or check formatting without changes:

```bash
make check-format
```

### EditorConfig

We use `.editorconfig` to maintain consistent coding styles across different editors and IDEs.

## Testing

### Running Tests

```bash
# Run all tests
make test

# Run specific test
make run-debug
make run-info
```

### Writing Tests

- Add tests for new functionality
- Ensure all tests pass before submitting PR
- Use descriptive test names
- Follow the existing test structure

## Documentation

### Code Documentation

- Use Doxygen-style comments for all public APIs
- Document parameters, return values, and exceptions
- Include usage examples in comments

### Generating Documentation

```bash
make docs
```

Documentation will be generated in `docs/html/`.

### README Updates

- Update README.md for new features
- Include usage examples
- Update installation instructions if needed

## Pull Request Process

1. **Create a feature branch** from `main`
2. **Make your changes** following the code style guidelines
3. **Test your changes** thoroughly
4. **Update documentation** if needed
5. **Commit your changes** with clear commit messages
6. **Push to your fork** and create a pull request

### Commit Message Format

Use conventional commit format:

```
type(scope): description

[optional body]

[optional footer]
```

Types:
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation changes
- `style`: Code style changes
- `refactor`: Code refactoring
- `test`: Test changes
- `chore`: Build/tooling changes

### Pull Request Checklist

- [ ] Code follows the style guidelines
- [ ] Self-review of code has been completed
- [ ] Code has been tested
- [ ] Documentation has been updated
- [ ] Commit messages follow conventional format
- [ ] All tests pass
- [ ] No merge conflicts

## Release Process

### Versioning

We follow [Semantic Versioning](https://semver.org/):

- **MAJOR**: Incompatible API changes
- **MINOR**: New functionality in a backwards compatible manner
- **PATCH**: Backwards compatible bug fixes

### Release Steps

1. Update version numbers in:
   - `package.xml`
   - `CMakeLists.txt`
   - `Doxyfile`
   - `README.md`

2. Create a release tag:
   ```bash
   git tag -a v1.0.0 -m "Release version 1.0.0"
   git push origin v1.0.0
   ```

3. Create a GitHub release with:
   - Release notes
   - Binary packages (if applicable)
   - Documentation links

## Getting Help

- Open an issue for bugs or feature requests
- Join our discussions for questions
- Check existing issues and pull requests

## License

By contributing to this project, you agree that your contributions will be licensed under the MIT License.

---

Thank you for contributing to Quill Logger for ROS2! 