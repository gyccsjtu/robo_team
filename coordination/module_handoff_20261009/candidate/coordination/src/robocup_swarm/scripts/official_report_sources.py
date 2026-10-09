"""Keep official coordinates bound to a fresh, ready camera source."""


class OfficialReportSources:
    def __init__(self, core_factory, readiness):
        self.core_factory = core_factory
        self.readiness = readiness
        self.cores = {}
        self.selected = {}
        self.eliminated = set()
        self.last_originals = {}

    def observe(self, observation, qualified):
        tag = observation['target_id']
        if not qualified or tag in self.eliminated or observation.get('schema_version') != 3:
            return False
        key = (tag, observation['uav_id'])
        if observation['sample_s'] <= self.last_originals.get(key, float('-inf')):
            return False
        core = self.cores.get(key)
        if core is None:
            core = self.cores[key] = self.core_factory()
        x, y, _ = observation['xyz']
        accepted = core.report(observation['sample_s'], tag, x, y, observation['confidence'],
                               observation['uav_id'], observation['observation_id'])
        if accepted:
            self.last_originals[key] = observation['sample_s']
        return accepted

    def allowed(self, tag, uid, stamp, now):
        source = self.readiness.sources.get((tag, uid))
        return bool(tag not in self.eliminated and source and source['ready']
                    and source['stamp'] == stamp
                    and 0 <= now-source['stamp'] <= 1.
                    and 0 <= now-stamp <= 1.)

    def tick(self, now):
        choices = {}
        for (tag, uid), core in self.cores.items():
            if tag in self.eliminated:
                continue
            track = core.tracks[tag]
            events = core.tick(now)
            if not self.allowed(tag, uid, track.t_obs, now):
                continue
            for event in events:
                if event['tag'] == tag and not event['eliminated']:
                    choices.setdefault(tag, {})[uid] = (event, track)
        selected = []
        for tag, sources in choices.items():
            uid = self.selected.get(tag)
            if uid not in sources:
                uid = max(sources, key=lambda source: (sources[source][1].t_obs, source))
                self.selected[tag] = uid
            event, track = sources[uid]
            selected.append((uid, event, track))
        return selected

    def eliminate(self, tag):
        self.eliminated.add(tag)
        self.selected.pop(tag, None)
        self.readiness.clear(tag)
        self.cores = {key: value for key, value in self.cores.items() if key[0] != tag}
        self.last_originals = {key: value for key, value in self.last_originals.items() if key[0] != tag}
