# Prédiction de position par contracteurs équivalents

## Idée générale
On modélise la position du bateau par une boîte d’intervalles et un contracteur `equivalent_contractor`.
Le contracteur est mis à jour par trois sources d’information :
1) Mouvement (dead reckoning) via un `CtcInverse`.
2) Mesure GPS via intersection.
3) Distances inter‑bateaux via `CtcInverse` sur la contrainte de distance.

## Formules utilisées pour les `CtcInverse`
### 1) Mouvement (dead reckoning, modèle inverse)
On note $x=(x,y)$ la position courante et $(\Delta x,\Delta y)$ le déplacement estimé (intervalles).
Le modèle « direct » est :
$$
x^+ = x + (\Delta x,\Delta y)
$$
Le contracteur inverse est construit à partir de la fonction :
$$
f_{\text{back}}(x) = \begin{bmatrix}x-\Delta x\\x-\Delta y\end{bmatrix}
$$
et impose que $f_{\text{back}}(x)$ appartienne à la boîte précédente (contracteur courant).
Depuis Codac 2, on peut passer directement un contracteur comme antécédent dans `CtcInverse` (plutôt qu’une simple boîte), ce qui permet de chaîner le contracteur courant comme contrainte amont.
En pratique, cela revient à restreindre les positions possibles $x$ telles que :
$$
x-\Delta x \in X\quad\text{et}\quad y-\Delta y \in Y
$$
où $X\times Y$ est la boîte contractée avant la mise à jour.

### 2) Distance inter‑bateaux
Soit un voisin de position incertaine $b=(b_x,b_y)$ (intervalles) et une mesure de distance $d$ (intervalle).
La contrainte est :
$$
\sqrt{(x-b_x)^2+(y-b_y)^2}\in d
$$
On construit alors :
$$
f_{\text{dist}}(x)=\sqrt{(x-b_x)^2+(y-b_y)^2}
$$
et `CtcInverse(f_dist, d)` contracte la boîte de $x$ pour satisfaire cette contrainte.

## Prédiction d’un bateau (pseudo‑algorithme)
Entrées :
- `box` : boîte courante de la position.
- `ctc` : contracteur courant.
- `dx`, `dy` : déplacement estimé sous forme d’intervalles.
- `gps_box` (optionnel) : boîte GPS.
- `neighbor_boxes` + `dist_intervals` : boîtes des voisins et distances mesurées.

Étapes :
1) **Mouvement**
   - Construire `dx_interval`, `dy_interval`.
   - Appliquer `add_movement_condition(dx_interval, dy_interval)`.
   - Mettre à jour `box` en y ajoutant `dx_interval`, `dy_interval`.
2) **GPS (si disponible)**
   - Appliquer `add_gps_condition(gps_box)`.
3) **Distances**
   - Pour chaque voisin :
     - Construire `dist_interval`.
     - Appliquer `add_distance_condition(dist_interval, neighbor_box)`.
4) **Contraction finale**
   - Appeler `get_box()` pour contracter `box` avec `ctc`.

## Ordre des opérations dans une flotte
Pour chaque période de mise à jour (ex. toutes les 2 s) :

1) **Prédiction locale (chaque bateau)**
   - Calculer le déplacement entre l’ancienne position et la nouvelle.
   - Appliquer `add_movement_condition`.

2) **Corrections locales**
   - Si un bateau a GPS : `add_gps_condition`.

3) **Diffusion**
   - Chaque bateau partage sa boîte contractée `get_box()`.

4) **Corrections par distance**
   - Chaque bateau applique les contraintes de distance vers les boîtes reçues.

5) **Contraction finale**
   - Appeler `get_box()` pour mettre à jour la boîte après toutes les contraintes.

6) **(Optionnel) Visualisation**
   - Paver le contracteur et afficher les boîtes.
