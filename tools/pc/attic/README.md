# `attic/` — scripts PC d'une phase revolue

Conserves pour l'historique, **plus utilises**. Ils restent versionnes : si l'un
redevient utile, il suffit de le remonter d'un niveau.

## Lanceurs « du Bureau » — abandonnes le 2026-06-26

`roby_1_init.sh`, `roby_2_prep.sh`, `roby_3_demo.sh` (+ `ROBY_DEMARRAGE.txt` reste
dans le home). Sequence de demarrage en trois clics depuis le Bureau, pour la demo.

Remplaces par les skills `/roby-lancer-bras`, `/roby-nid` et
`/roby-enregistrer-episodes`, qui font la meme chose **avec les garde-fous** :
balayage des deux machines, GATE anti-mock (`RobySystem` vs `FakeSystem`), GATE
anti-race sur `/robot_description`, et sequence de verrouillage correcte autour du
nid. Les lanceurs du Bureau n'ont aucun de ces controles — c'est precisement
pourquoi ils ont ete abandonnes.

⚠️ Ils ont ete versionnes par erreur dans `tools/pc/` lors de la mise sous git du
2026-07-20 : ils faisaient partie des 48 scripts du home et n'ont pas ete reconnus
comme obsoletes sur le moment.
