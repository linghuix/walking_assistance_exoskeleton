#include <cstdio>
#include <algorithm>
#include <iostream>
#include <cstring>
#include <vector>
using namespace std;
const int maxn = 33;
const int inf = 0x3f3f3f3f;
const int dr[] = { -2, -2, -1, -1,  1,  1,  2,  2 };
const int dc[] = { -1,  1, -2,  2, -2,  2, -1,  1 };

int n, ok, vis[maxn*maxn];

void dfs(int pos, int step) {
    if (step == n*n) {
        ok = 1;
        for (int r = 1; r <= n; r ++) {
            for (int c = 1; c <= n; c ++)
                printf ("%3d", vis[(r-1)*n+c]);
            printf ("\n");
        }
        return;
    }

    int r = (pos-1)/n + 1;
    int c = pos % n;
    if (!c) c = n;

    for (int i = 0; i < 8; i++) {
        int rr = r + dr[i], cc = c + dc[i], pos1 = (rr-1)*n+cc;
        if (rr < 1 || cc < 1 || rr > n || cc > n || vis[pos1]) continue;
        vis[pos1] = vis[pos] + 1;
        dfs(pos1, step + 1);  if (ok) return;
         
        vis[pos1] = 0;
    }
}

int main() {
    int T, r, c; cin >> T;
    for (int tt = 1; tt <= T; tt ++) {
        cin >> n >> r >> c;
		//r,c,∆Â≈Ã∏Ò ˝
		cout<<"n="<<n<<endl;
		cout<<"r="<<r<<endl;
		cout<<"c="<<c<<endl;
        memset(vis, 0, sizeof vis);
        vis[(r-1)*n+c] = 1;
        ok = 0;
        printf("Case #%d:\n", tt);
        dfs((r-1)*n+c, 1);
        if (!ok) printf ("No solution.\n");
    }
	return 0;
}
