#!/bin/bash

set -e

# 1. input nama branch
read -p "Masukkan nama branch: " BRANCH

# 1a. cek apakah branch sudah ada
if git show-ref --verify --quiet refs/heads/"$BRANCH"; then
    echo "Branch '$BRANCH' sudah ada. Beralih ke branch tersebut."
    git checkout --quiet "$BRANCH"
else
    # 1b. jika belum ada, pilih apakah ingin membuat branch baru
    read -p "Branch '$BRANCH' tidak ditemukan. Apakah Anda ingin membuat branch baru? (y/n): " CREATE_BRANCH
    if [[ "$CREATE_BRANCH" =~ ^[Yy]$ ]]; then
        echo "Membuat branch baru '$BRANCH'..."
        git checkout -b "$BRANCH"
        echo "Branch '$BRANCH' telah dibuat dan beralih ke branch tersebut."
    else
        echo "Branch tidak dibuat. Keluar dari skrip."
        exit 1
    fi
fi

# 2. lakukan git commit
read -p "Masukkan pesan commit: " COMMIT_MSG
if [ -z "$COMMIT_MSG" ]; then
    echo "Pesan commit tidak boleh kosong."
    exit 1
fi

git add .
git commit -m "$COMMIT_MSG"

# 3. cek apakah ada perubahan pada remote
if git fetch origin "$BRANCH" --quiet; then
    LOCAL_COMMIT=$(git rev-parse "$BRANCH")
    REMOTE_COMMIT=$(git rev-parse "origin/$BRANCH")
    BASE_COMMIT=$(git merge-base "$BRANCH" "origin/$BRANCH")

    if [ "$REMOTE_COMMIT" = "$BASE_COMMIT" ]; then
        # remote tidak lebih maju dari lokal
        echo "Tidak ada perubahan baru pada remote untuk branch '$BRANCH'."
    elif [ "$LOCAL_COMMIT" = "$BASE_COMMIT" ]; then
        # remote lebih maju dari lokal
        echo "Remote lebih maju dari lokal pada branch '$BRANCH'."
        read -p "Apakah Anda ingin menarik perubahan terbaru dari remote? (y/n): " PULL_CHOICE
        if [[ "$PULL_CHOICE" =~ ^[Yy]$ ]]; then
            if git pull --rebase --quiet origin "$BRANCH"; then
                echo "Perubahan terbaru telah ditarik dari branch '$BRANCH'."
            else
                echo "Gagal menarik perubahan dari branch '$BRANCH'."
                exit 1
            fi
        else
            echo "Tidak menarik perubahan dari remote. Push mungkin gagal jika remote lebih baru."
        fi
    else
        echo "Branch lokal dan remote telah berbeda (diverged)."
        echo "Silakan lakukan merge/rebase secara manual."
        exit 1
    fi
else
    echo "Gagal mengambil informasi dari remote. Pastikan koneksi internet Anda berfungsi."
    exit 1
fi

# 4. push ke remote
if git push origin "$BRANCH"; then
    echo "Perubahan telah berhasil di-push ke branch '$BRANCH'."
else
    echo "Gagal melakukan push ke branch '$BRANCH'."
    exit 1
fi
