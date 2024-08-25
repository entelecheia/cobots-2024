# 협동로봇활용 2024

[![halla-img]][halla-url]
[![course-img]][course-url]
[![lecture-img]][lecture-url]
[![pypi-image]][pypi-url]
[![version-image]][release-url]
[![release-date-image]][release-url]
[![license-image]][license-url]
[![codecov][codecov-image]][codecov-url]
[![jupyter-book-image]][docs-url]

<!-- Links: -->

[halla-img]: https://img.shields.io/badge/CHU-halla.ai-blue
[halla-url]: https://halla.ai
[course-img]: https://img.shields.io/badge/course-entelecheia.ai-blue
[course-url]: https://course.entelecheia.ai
[lecture-img]: https://img.shields.io/badge/lecture-entelecheia.ai-blue
[lecture-url]: https://lecture.entelecheia.ai
[codecov-image]: https://codecov.io/gh/entelecheia/cobots-2024/branch/main/graph/badge.svg?token=IFGaRC86K7
[codecov-url]: https://codecov.io/gh/entelecheia/cobots-2024
[pypi-image]: https://img.shields.io/pypi/v/cobots2024
[license-image]: https://img.shields.io/github/license/entelecheia/cobots-2024
[license-url]: https://github.com/entelecheia/cobots-2024/blob/main/LICENSE
[version-image]: https://img.shields.io/github/v/release/entelecheia/cobots-2024?sort=semver
[release-date-image]: https://img.shields.io/github/release-date/entelecheia/cobots-2024
[release-url]: https://github.com/entelecheia/cobots-2024/releases
[jupyter-book-image]: https://jupyterbook.org/en/stable/_images/badge.svg
[repo-url]: https://github.com/entelecheia/cobots-2024
[pypi-url]: https://pypi.org/project/cobots2024
[docs-url]: https://cobots2024.jeju.ai
[changelog]: https://github.com/entelecheia/cobots-2024/blob/main/CHANGELOG.md
[contributing guidelines]: https://github.com/entelecheia/cobots-2024/blob/main/CONTRIBUTING.md

<!-- Links: -->

Collaborative Robot Applications

- Documentation: [https://cobots2024.jeju.ai][docs-url]
- GitHub: [https://github.com/entelecheia/cobots-2024][repo-url]
- PyPI: [https://pypi.org/project/cobots2024][pypi-url]

이 과정은 협동로봇의 기본적인 이해부터 고급 프로그래밍 기술까지 다루며, 학생들이 협동로봇을 활용한 다양한 작업 공정을 설계하고 구현할 수 있는 능력을 배양하는 것을 목표로 합니다. 로봇 시스템 설치, 조작, 프로그래밍, 그리고 실제 작업 공정 구현까지의 전 과정을 학습합니다.

## Installation

To install the Cobots 2024 package, use the following command:

```
pip install cobots2024
```

Or

```
pip install --user cobots2024
```

The `--user` flag is optional and can be used to install the package in the user's home directory instead of the system-wide location.

## Usage

To use the Cobots 2024 CLI, run the following command:

```
cobots2024 [OPTIONS]
```

If no option is provided, the website of the book will open in the default web browser.

### Options

The following options are available:

- `--version`: Show the version of the package and exit.
- `-b`, `--build`: Build the book.
- `-l`, `--local`: Open the locally built HTML version of the book in the browser.
- `--help`: Show the help message and exit.

### Examples

1. To build the book, use the following command:

   ```
   cobots2024 --build
   ```

   This will trigger the build process for the book.

2. To open the locally built HTML version of the book in the browser, use the following command:

   ```
   cobots2024 --local
   ```

   This will open the book's HTML file in your default web browser.

3. To view the version of the package, use the following command:

   ```
   cobots2024 --version
   ```

   This will display the version number of the Cobots 2024 package.

For more information and additional options, run `cobots2024 --help` to see the help message.

## Contributors

## Changelog

See the [CHANGELOG] for more information.

## Contributing

Contributions are welcome! Please see the [contributing guidelines] for more information.

## License

This project is released under the [CC-BY-4.0 License][license-url].
