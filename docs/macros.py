def define_env(env):
    from datetime import datetime, timezone
    # Lokaler Zeitstempel für den Footer/Seiten
    env.variables['build_time'] = datetime.now(timezone.utc).astimezone().strftime('%d.%m.%Y %H:%M')
