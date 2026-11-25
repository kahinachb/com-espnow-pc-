import cv2
import serial
import multiprocessing as mp
import queue
import threading
import time
import csv
import argparse
import struct
from pathlib import Path
from threading import BrokenBarrierError

BARRIER_TIMEOUT_S = 2.0
WARMUP_FRAMES = 15


def ecoute_esp32(etat_switch, flag_arret, trigger_log_path, bin_path):
    """Processus 1 : Enregistrement brut continu à 2 000 000 bauds"""
    ser = serial.Serial('/dev/ttyACM0', 2000000, timeout=0.01)
    buffer = bytearray()
    trigger_events = []
    derniere_valeur_loggee = None

    with open(bin_path, 'wb') as f_bin:
        while not flag_arret.value:
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                f_bin.write(chunk)
                f_bin.flush()

                buffer.extend(chunk)
                if len(buffer) > 500:
                    buffer = buffer[-500:]

                for i in range(len(buffer) - 3, -1, -1):
                    if buffer[i] in (0, 1):
                        if (buffer[i+1] == 1 and buffer[i+2] == 2) or \
                           (buffer[i+1] == 9 and buffer[i+2] == 10):
                            valeur = buffer[i]
                            with etat_switch.get_lock():
                                etat_switch.value = valeur

                            # N'enregistre que les transitions (0->1, 1->0),
                            # avec l'horloge murale PC : sert d'ancre commune
                            # pour recaler les videos (elles aussi horodatees
                            # avec time.time() dans capture_camera) sur le
                            # meme instant de declenchement que les semelles.
                            if valeur != derniere_valeur_loggee:
                                trigger_events.append((time.time(), valeur))
                                derniere_valeur_loggee = valeur

                            buffer = buffer[i:]
                            break
    ser.close()

    with open(trigger_log_path, 'w', newline='') as f_log:
        writer = csv.writer(f_log)
        writer.writerow(['timestamp_s', 'switch_value'])
        writer.writerows(trigger_events)


def capture_camera(path_cam, nom_fichier, etat_switch, flag_arret, barrier):
    """Processus dédié à une caméra (sans aucune file d'affichage ni RAM superflue)

    barrier est partagee UNIQUEMENT entre les deux processus camera (pas avec
    l'ESP32) : elle sert a synchroniser l'instant de capture (cap.grab()) des
    deux cameras entre elles, sans aucun impact sur ecoute_esp32 qui tourne
    dans un processus completement separe et ne connait meme pas la barriere.
    """
    # Tout est dans ce try/finally des l'ouverture de la camera : si CETTE
    # camera echoue a s'ouvrir (mauvais /dev/videoX, deja utilise, etc.), le
    # finally s'execute quand meme et appelle barrier.abort(), pour ne pas
    # laisser l'AUTRE camera bloquee a attendre indefiniment (jusqu'au
    # timeout) un partenaire qui ne viendra jamais. Avant ce correctif, une
    # erreur ici (hors du try/finally) faisait planter le process sans
    # debloquer la barriere ni ecrire le csv de timestamps : l'autre camera
    # finissait par timeout sans avoir capture une seule frame.
    cap = None
    out = None
    writer_thread = None
    frame_queue: "queue.Queue" = queue.Queue(maxsize=150)
    frame_timestamps = []
    dropped_frames = 0

    def writer_loop():
        frame_index = 0
        while True:
            item = frame_queue.get()
            if item is None:
                break
            frame, timestamp = item
            out.write(frame)
            frame_timestamps.append((frame_index, timestamp))
            frame_index += 1

    try:
        # Utilisation du backend V4L2 pour une stabilité maximale sous Linux
        cap = cv2.VideoCapture(path_cam, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f"impossible d'ouvrir la camera {path_cam}")
        # Sans ceci, le driver negocie souvent YUYV (non compresse), qui
        # plafonne a 30 fps en 640x480 sur cette camera (verifie avec
        # v4l2-ctl --list-formats-ext) ; MJPG (compresse) permet d'atteindre
        # les 40 fps demandes ci-dessous, y compris en 1280x720. A definir
        # AVANT CAP_PROP_FPS.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        # Resolution/fps calibres via guvcview (exposition, etc.) : ces
        # reglages sont au niveau pilote (persistants), mais la resolution
        # et le format sont renegocies a chaque ouverture du device par une
        # application -> on les redemande explicitement ici pour ne pas
        # dependre de ce qu'une autre appli a laisse en dernier.
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cap.set(cv2.CAP_PROP_FPS, 40)
        # BUFFERSIZE=1 a ete teste puis retire : ca forcait un cycle
        # grab/attente totalement synchrone (aucun pipeline cote driver), ce
        # qui plafonnait le debit reel a ~20 fps sur cette camera au lieu de
        # 40. Avec le buffer par defaut, cap.grab() peut renvoyer une frame
        # legerement plus ancienne (le temps de quelques buffers), mais ce
        # n'est pas un probleme ici : la barriere garde les deux cameras
        # verrouillees en cadence l'une sur l'autre (impossible que l'une
        # prenne de l'avance/du retard sur l'autre sans que ca bloque la
        # boucle), donc ce petit retard reste comparable des deux cotes au
        # lieu de creer une derive progressive entre cam1 et cam2.
        w, h = int(cap.get(3)), int(cap.get(4))
        if w == 0 or h == 0:
            raise RuntimeError(f"resolution invalide renvoyee par {path_cam} ({w}x{h})")

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(nom_fichier, fourcc, 40.0, (w, h))
        if not out.isOpened():
            raise RuntimeError(f"impossible de creer le fichier video {nom_fichier}")

        # Quelques premieres captures ignorees : juste apres l'ouverture, le
        # driver a besoin d'un court instant pour se stabiliser sur le
        # nouveau format/resolution/fps (les toutes premieres frames sont
        # souvent plus lentes, voire incompletes). Elles passent quand meme
        # par la barriere pour rester synchrones avec l'autre camera des le
        # debut, mais ne sont ni ecrites ni horodatees.
        for _ in range(WARMUP_FRAMES):
            if flag_arret.value:
                break
            try:
                barrier.wait(timeout=BARRIER_TIMEOUT_S)
            except BrokenBarrierError:
                break
            cap.grab()
            try:
                barrier.wait(timeout=BARRIER_TIMEOUT_S)
            except BrokenBarrierError:
                break
            cap.retrieve()

        # L'ecriture disque (out.write, encodage mp4v) reste decouplee de la
        # capture par une file + un thread dedie : un ralentissement
        # d'ecriture ne doit jamais retarder cap.grab()/cap.retrieve(), sous
        # peine de desynchroniser cette camera de l'autre.
        writer_thread = threading.Thread(target=writer_loop, daemon=True)
        writer_thread.start()

        while not flag_arret.value:
            # --- 1) les deux cameras attendent ici avant de capturer, pour
            # declencher cap.grab() au meme instant sur les deux.
            try:
                barrier.wait(timeout=BARRIER_TIMEOUT_S)
            except BrokenBarrierError:
                break

            # --- 2) grab() est rapide (demande juste au driver de capturer
            # l'image courante) ; le decodage plus lent (retrieve) vient
            # apres, une fois que les deux cameras ont grab().
            cap.grab()

            try:
                barrier.wait(timeout=BARRIER_TIMEOUT_S)
            except BrokenBarrierError:
                break

            ret, frame = cap.retrieve()
            if not ret:
                continue

            with etat_switch.get_lock():
                valeur_actuelle = etat_switch.value

            # Écriture dans la vidéo uniquement si le switch est à 1
            if valeur_actuelle == 1:
                # Horodatage au moment de la capture (grab/retrieve), pas de
                # l'ecriture, avec l'horloge murale PC : reference reelle par
                # frame, comparable a trigger_log.csv et aux autres cameras.
                timestamp = time.time()
                try:
                    frame_queue.put_nowait((frame, timestamp))
                except queue.Full:
                    # Ecriture disque trop lente pour suivre : on prefere
                    # perdre cette frame plutot que de bloquer la boucle et
                    # de decaler la camera par rapport a l'autre.
                    dropped_frames += 1
    except Exception as exc:  # noqa: BLE001
        print(f"[{nom_fichier}] ERREUR : {exc}")
    finally:
        # Debloque immediatement l'autre camera si elle est encore en train
        # d'attendre a la barriere (au lieu de la laisser patienter jusqu'au
        # timeout), pour un arret propre et rapide des deux cotes, meme si
        # CETTE camera a echoue avant d'avoir capture la moindre frame.
        barrier.abort()

        if writer_thread is not None:
            frame_queue.put(None)
            writer_thread.join()

        if cap is not None:
            cap.release()
        if out is not None:
            out.release()

        timestamps_path = Path(nom_fichier).with_name(
            Path(nom_fichier).stem + '_timestamps.csv'
        )
        with open(timestamps_path, 'w', newline='') as f_ts:
            writer = csv.writer(f_ts)
            writer.writerow(['frame_index', 'timestamp_s'])
            writer.writerows(frame_timestamps)

        if dropped_frames:
            print(f"[{nom_fichier}] attention : {dropped_frames} frame(s) perdue(s) (ecriture disque trop lente)")
        print(f"[{nom_fichier}] {len(frame_timestamps)} frames enregistrees.")


# ---------------------------------------------------------------------------
# Diagnostic de fin d'acquisition (calcule APRES l'arret, en relisant les
# fichiers ecrits : aucun impact sur les boucles d'acquisition ci-dessus).
# Meme logique que bertec-forceplate-linux/bertec_record_wrench_csv_multiprocess.py
# ---------------------------------------------------------------------------
FRAME_SIZE = 36
PACKET_SIZE = 148
SERIAL_RECORD_SIZE = PACKET_SIZE + 1
PACKET_SEQUENCES = {
    (1, 2): ("left", ((1, 2), (3, 4), (5, 6), (7, 8))),
    (9, 10): ("right", ((9, 10), (11, 12), (13, 14), (15, 16))),
}
CAMERA_FPS = 40.0
# Ecart relatif max toleré entre le nombre de paquets gauche et droite (en %)
ECART_GD_MAX_PCT = 2.0


def inspect_insole_packet(data, start):
    """Retourne le pied si un paquet ESP-NOW complet commence a ``start``."""
    if start + PACKET_SIZE > len(data):
        return None
    sequence_info = PACKET_SEQUENCES.get(tuple(data[start:start + 2]))
    if sequence_info is None:
        return None
    foot, sequence = sequence_info
    for patch_index, identifier in enumerate(sequence):
        frame_start = start + patch_index * FRAME_SIZE
        if tuple(data[frame_start:frame_start + 2]) != identifier:
            return None
        forces = struct.unpack('>HHHH', data[frame_start + 3:frame_start + 11])
        if any(force > 0x3FFF for force in forces):
            return None
    return foot


def diagnostic_semelles(bin_path):
    """Relit le .bin et compte paquets valides, pertes, doublons, fragments."""
    stats = {
        'bytes_written': 0, 'packets': 0, 'left_packets': 0, 'right_packets': 0,
        'leading_bytes': 0, 'internal_fragment_bytes': 0, 'trailing_bytes': 0,
        'lost_packets': 0, 'duplicate_packets': 0, 'signal_changes': 0,
    }
    # Memes compteurs, restreints aux paquets recus pendant Signal=1
    # (periodes reellement enregistrees par les cameras).
    stats_on = {
        'packets': 0, 'left_packets': 0, 'right_packets': 0,
        'internal_fragment_bytes': 0, 'lost_packets': 0, 'duplicate_packets': 0,
    }
    data = Path(bin_path).read_bytes()
    stats['bytes_written'] = len(data)
    last_cpt = {}
    last_cpt_on = {}
    last_signal = None
    have_seen_packet = False
    pos = 0

    def record(foot, packet_start, marker):
        nonlocal last_signal, have_seen_packet
        if marker is not None:
            if last_signal is not None and marker != last_signal:
                stats['signal_changes'] += 1
            last_signal = marker
        cpt = data[packet_start + 2]
        previous_cpt = last_cpt.get(foot)
        if previous_cpt is not None:
            step = (cpt - previous_cpt) & 0xFF
            if step == 0:
                stats['duplicate_packets'] += 1
            elif 1 < step < 128:
                stats['lost_packets'] += step - 1
        last_cpt[foot] = cpt
        stats['packets'] += 1
        stats['left_packets' if foot == 'left' else 'right_packets'] += 1
        have_seen_packet = True

        if last_signal == 1:
            # Pertes comptees seulement entre deux paquets consecutifs du
            # meme pied tous deux recus pendant Signal=1 (le passage 0->1
            # ne compte pas comme une perte).
            previous_cpt_on = last_cpt_on.get(foot)
            if previous_cpt_on is not None:
                step = (cpt - previous_cpt_on) & 0xFF
                if step == 0:
                    stats_on['duplicate_packets'] += 1
                elif 1 < step < 128:
                    stats_on['lost_packets'] += step - 1
            last_cpt_on[foot] = cpt
            stats_on['packets'] += 1
            stats_on['left_packets' if foot == 'left' else 'right_packets'] += 1
        else:
            last_cpt_on.clear()

    while len(data) - pos >= PACKET_SIZE:
        foot = inspect_insole_packet(data, pos)
        if foot is not None:
            record(foot, pos, None)
            pos += PACKET_SIZE
            continue
        if data[pos] in (0, 1):
            if len(data) - pos < SERIAL_RECORD_SIZE:
                break
            foot = inspect_insole_packet(data, pos + 1)
            if foot is not None:
                record(foot, pos + 1, data[pos])
                pos += SERIAL_RECORD_SIZE
                continue
        if have_seen_packet:
            stats['internal_fragment_bytes'] += 1
            if last_signal == 1:
                stats_on['internal_fragment_bytes'] += 1
        else:
            stats['leading_bytes'] += 1
        pos += 1

    stats['trailing_bytes'] = len(data) - pos
    return stats, stats_on


def lire_periodes_actives(trigger_log_path):
    """Liste des intervalles [debut, fin] ou le switch etait a 1."""
    periodes = []
    debut = None
    with open(trigger_log_path, newline='') as f:
        for row in csv.DictReader(f):
            t, v = float(row['timestamp_s']), int(row['switch_value'])
            if v == 1 and debut is None:
                debut = t
            elif v == 0 and debut is not None:
                periodes.append((debut, t))
                debut = None
    if debut is not None:
        periodes.append((debut, float('inf')))
    return periodes


def diagnostic_camera(timestamps_path, periodes):
    """fps effectif et trous entre frames, calcules a l'interieur de chaque
    periode switch=1 (les pauses switch=0 ne comptent pas comme des pertes)."""
    with open(timestamps_path, newline='') as f:
        ts = [float(row['timestamp_s']) for row in csv.DictReader(f)]
    res = {'frames': len(ts), 'fps': 0.0, 'max_gap_ms': 0.0,
           'gaps': 0, 'missing_frames_est': 0}
    if len(ts) < 2:
        return res
    periode_nominale = 1.0 / CAMERA_FPS
    duree_totale = 0.0
    n_intervalles = 0
    for i in range(1, len(ts)):
        dt = ts[i] - ts[i - 1]
        meme_periode = any(a <= ts[i - 1] and ts[i] <= b for a, b in periodes) \
            if periodes else True
        if not meme_periode:
            continue
        duree_totale += dt
        n_intervalles += 1
        res['max_gap_ms'] = max(res['max_gap_ms'], dt * 1000)
        if dt > 1.5 * periode_nominale:
            res['gaps'] += 1
            res['missing_frames_est'] += round(dt / periode_nominale) - 1
    if duree_totale > 0:
        res['fps'] = n_intervalles / duree_totale
    return res


def afficher_diagnostic(bin_path, trigger_log_path, cam_paths):
    print("\n==================== DIAGNOSTIC ====================")
    try:
        stats, stats_on = diagnostic_semelles(bin_path)
        print(
            "Semelles : "
            f"{stats['bytes_written']} octets, {stats['packets']} paquets valides "
            f"(G={stats['left_packets']}, D={stats['right_packets']})"
        )
        print(
            "Diagnostic semelles : "
            f"fragments internes={stats['internal_fragment_bytes']} octet(s), "
            f"paquets perdus={stats['lost_packets']}, "
            f"doublons={stats['duplicate_packets']}, "
            f"changements Signal={stats['signal_changes']}, "
            f"début partiel={stats['leading_bytes']} octet(s), "
            f"fin partielle={stats['trailing_bytes']} octet(s)"
        )
        print(
            "Semelles pendant Signal=1 : "
            f"{stats_on['packets']} paquets valides "
            f"(G={stats_on['left_packets']}, D={stats_on['right_packets']}), "
            f"fragments internes={stats_on['internal_fragment_bytes']} octet(s), "
            f"paquets perdus={stats_on['lost_packets']}, "
            f"doublons={stats_on['duplicate_packets']}"
        )
        if stats_on['packets'] == 0:
            print("  -> ATTENTION : aucun paquet reçu avec Signal=1")
        elif stats_on['left_packets'] == 0 or stats_on['right_packets'] == 0:
            manquant = 'gauche' if stats_on['left_packets'] == 0 else 'droite'
            print(f"  -> ATTENTION : aucun paquet de la semelle {manquant} pendant Signal=1")
        elif stats_on['internal_fragment_bytes'] or stats_on['lost_packets'] or stats_on['duplicate_packets']:
            print("  -> ATTENTION : pertes/corruptions pendant l'enregistrement (Signal=1)")
        else:
            print("  -> OK : aucune perte pendant l'enregistrement (Signal=1)")

        g, d = stats_on['left_packets'], stats_on['right_packets']
        if g and d:
            ecart = g - d
            ecart_pct = 100.0 * abs(ecart) / max(g, d)
            print(
                f"Écart gauche/droite (Signal=1) : G-D={ecart:+d} paquet(s) "
                f"({ecart_pct:.2f} %, seuil {ECART_GD_MAX_PCT:.1f} %)"
            )
            if ecart_pct > ECART_GD_MAX_PCT:
                moins = 'gauche' if g < d else 'droite'
                print(f"  -> ATTENTION : la semelle {moins} a envoyé nettement moins de paquets")
    except Exception as exc:  # noqa: BLE001
        print(f"Semelles : diagnostic impossible ({exc})")

    try:
        periodes = lire_periodes_actives(trigger_log_path)
        print(f"Trigger : {len(periodes)} période(s) d'enregistrement (switch=1)")
    except Exception as exc:  # noqa: BLE001
        periodes = []
        print(f"Trigger : lecture impossible ({exc})")

    for cam_path in cam_paths:
        timestamps_path = Path(cam_path).with_name(Path(cam_path).stem + '_timestamps.csv')
        try:
            r = diagnostic_camera(timestamps_path, periodes)
            print(
                f"{Path(cam_path).name} : {r['frames']} frames, "
                f"fps effectif={r['fps']:.1f} (nominal {CAMERA_FPS:.0f}), "
                f"trous={r['gaps']}, frames manquantes estimées={r['missing_frames_est']}, "
                f"écart max={r['max_gap_ms']:.1f} ms"
            )
        except Exception as exc:  # noqa: BLE001
            print(f"{Path(cam_path).name} : diagnostic impossible ({exc})")
    print("====================================================")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', default='.',
                        help="dossier ou enregistrer les donnees (ex: sujetA/squat)")
    args = parser.parse_args()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Variables partagées
    etat_switch = mp.Value('i', 0)
    flag_arret = mp.Value('b', False)
    trigger_log_path = str(output_dir / 'trigger_log.csv')
    bin_path = str(output_dir / 'dataSemelle.bin')
    cam1_path = str(output_dir / 'cam1_manip.mp4')
    cam2_path = str(output_dir / 'cam2_manip.mp4')

    # Démarrage des processus (Semelles + Caméras sur leurs ports respectifs)
    print("Démarrage de l'acquisition (Mode ultra-léger sans affichage)...")

    # Barriere partagee UNIQUEMENT entre les 2 processus camera (2 parties) :
    # elle synchronise l'instant de cap.grab() entre cam1 et cam2. L'ESP32
    # n'y participe pas et n'a meme pas connaissance de son existence.
    camera_barrier = mp.Barrier(2)

    p_semelles = mp.Process(
        target=ecoute_esp32, args=(etat_switch, flag_arret, trigger_log_path, bin_path)
    )

    # Adaptez les chemins selon vos ports (ex: /dev/video0 et /dev/video3)
    p_cam1 = mp.Process(
        target=capture_camera,
        args=('/dev/video2', cam1_path, etat_switch, flag_arret, camera_barrier),
    )
    p_cam2 = mp.Process(
        target=capture_camera,
        args=('/dev/video4', cam2_path, etat_switch, flag_arret, camera_barrier),
    )

    p_semelles.start()
    p_cam1.start()
    p_cam2.start()

    print("\n---------------------------------------------------")
    print(f" ACQUISITION EN COURS DANS LE DOSSIER {output_dir} :")
    print(" - Semelles   : dataSemelle.bin")
    print(" - Vidéos     : cam1_manip.mp4 & cam2_manip.mp4")
    print(" - Horodatage : trigger_log.csv, cam1_manip_timestamps.csv, cam2_manip_timestamps.csv")
    print(" (Le déclenchement se fait via le switch de l'ESP32)")
    print("---------------------------------------------------")

    try:
        # Le script attend sagement ici sans consommer de CPU
        input("Appuyez sur la touche [ENTRÉE] de votre clavier pour stopper...\n")
    except KeyboardInterrupt:
        pass

    finally:
        print("Arrêt des processus et fermeture des fichiers...")
        with flag_arret.get_lock():
            flag_arret.value = True

        p_semelles.join()
        p_cam1.join()
        p_cam2.join()
        print("Terminé ! Tous les fichiers sont prêts pour le post-traitement.")
        afficher_diagnostic(bin_path, trigger_log_path, [cam1_path, cam2_path])
