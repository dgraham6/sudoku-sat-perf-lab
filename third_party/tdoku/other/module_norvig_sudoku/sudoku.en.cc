#include <iostream>
#include <vector>
#include <algorithm>
#include <memory>

using namespace std;

class Possible {
    int _b;
public:
    Possible() : _b(0x1ff) {}
    bool is_on(int i) const { return _b & (1 << (i - 1)); }
    int count() const { return __builtin_popcount(_b); }
    void eliminate(int i) { _b &= ~(1 << (i - 1)); }
    int val() const { return  __builtin_ffs(_b); }
};

class Sudoku {
    vector<Possible> _cells;
    static vector<vector<int> > _group, _neighbors, _groups_of;
    bool _invalid = false;
    bool eliminate(int k, int val);

public:
    Sudoku(string s);
    static bool init();
    Possible possible(int k) const { return _cells[k]; }
    bool is_solved() const;
    bool is_invalid() const { return _invalid; };
    bool assign(int k, int val);
    int least_count() const;
    void write(char *solution) const;
};

bool Sudoku::is_solved() const {
    for (int k = 0; k < _cells.size(); k++) {
        if (_cells[k].count() != 1) {
            return false;
        }
    }
    return true;
}

void Sudoku::write(char *solution) const {
    for (int i = 0; i < 81; i++) {
        solution[i] = '0' + _cells[i].val();
    }
}

vector<vector<int> >
        Sudoku::_group(27), Sudoku::_neighbors(81), Sudoku::_groups_of(81);

bool Sudoku::init() {
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            const int k = i * 9 + j;
            const int x[3] = {i, 9 + j, 18 + (i / 3) * 3 + j / 3};
            for (int g = 0; g < 3; g++) {
                _group[x[g]].push_back(k);
                _groups_of[k].push_back(x[g]);
            }
        }
    }
    for (int k = 0; k < _neighbors.size(); k++) {
        for (int x = 0; x < _groups_of[k].size(); x++) {
            for (int j = 0; j < 9; j++) {
                int k2 = _group[_groups_of[k][x]][j];
                if (k2 != k) _neighbors[k].push_back(k2);
            }
        }
    }
    return true;
}

bool Sudoku::assign(int k, int val) {
    for (int i = 1; i <= 9; i++) {
        if (i != val) {
            if (!eliminate(k, i)) return false;
        }
    }
    return true;
}

bool Sudoku::eliminate(int k, int val) {
    if (!_cells[k].is_on(val)) {
        return true;
    }
    _cells[k].eliminate(val);
    const int N = _cells[k].count();
    if (N == 0) {
        return false;
    } else if (N == 1) {
        const int v = _cells[k].val();
        for (int i = 0; i < _neighbors[k].size(); i++) {
            if (!eliminate(_neighbors[k][i], v)) return false;
        }
    }
    for (int i = 0; i < _groups_of[k].size(); i++) {
        const int x = _groups_of[k][i];
        int n = 0, ks;
        for (int j = 0; j < 9; j++) {
            const int p = _group[x][j];
            if (_cells[p].is_on(val)) {
                n++, ks = p;
            }
        }
        if (n == 0) {
            return false;
        } else if (n == 1) {
            if (!assign(ks, val)) {
                return false;
            }
        }
    }
    return true;
}

int Sudoku::least_count() const {
    int k = -1, min = 10;
    for (int i = 0; i < _cells.size(); i++) {
        const int m = _cells[i].count();
        if (m != 1 && (k == -1 || m < min)) {
            min = m, k = i;
        }
    }
    return k;
}

Sudoku::Sudoku(string s) : _cells(81) {
    if (s.length() >= 729) {
        for (int i = 0; i < 81; i++) {
            for (int v = 0; v < 9; v++) {
                if (s[i * 9 + v] == '.') {
                    if (!eliminate(i, v + 1)) {
                        _invalid = true;
                        return;
                    }
                }
            }
        }
    } else {
        for (int i = 0; i < 81; i++) {
            if (s[i] >= '1' && s[i] <= '9') {
                if (!assign(i, s[i] - '0')) {
                    _invalid = true;
                    return;
                }
            }
        }
    }
}

size_t limit_ = 1;
size_t solutions_ = 0;
size_t guesses_ = 0;
unique_ptr<Sudoku> solution_(nullptr);

void solve(unique_ptr<Sudoku> S) {
    if (S->is_invalid()) {
        return;
    }
    if (S->is_solved()) {
        solutions_++;
        if (limit_ == 1) {
            solution_ = std::move(S);
        }
        return;
    }
    int k = S->least_count();
    Possible p = S->possible(k);
    int remaining = p.count();
    for (int i = 1; i <= 9; i++) {
        if (p.is_on(i)) {
            guesses_ += (remaining-- > 1);
            unique_ptr<Sudoku> S1(new Sudoku(*S));
            if (S1->assign(k, i)) {
                solve(std::move(S1));
                if (solutions_ == limit_) {
                    return;
                }
            }
        }
    }
}

extern "C"
size_t OtherSolverNorvig(const char *input, size_t limit, uint32_t /*unused_configuration*/,
                         char *solution, size_t *num_guesses) {
    static bool initialized = Sudoku::init();
    string puzzle(input);
    limit_ = limit;
    solutions_ = 0;
    guesses_ = 0;
    solution_ = nullptr;
    solve(unique_ptr<Sudoku>(new Sudoku(puzzle)));
    if (solution_) solution_->write(solution);
    *num_guesses = guesses_;
    return solutions_;
}
