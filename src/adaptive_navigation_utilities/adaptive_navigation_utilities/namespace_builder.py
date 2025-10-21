# namespace_builder.py
class Namespace:
    def __init__(self, *parts):
        self.parts = []
        self._sub(*parts)

    def _sub(self, *parts):
        for part in parts:
            if part:
                self.parts.extend(p for p in part.split('/') if p)
        
    def sub(self, *parts):
        self._sub(*parts)
        return self

    def remove(self, *parts_to_remove):
        # Remove specified parts (by name, not index)
        to_remove = set(p.strip('/') for p in parts_to_remove)
        self.parts = [p for p in self.parts if p not in to_remove]
        return self

    def build(self):
        return '/'.join(self.parts)
    
    def includes(self, name: str) -> bool:
        name = name.strip('/')
        return any(name in part for part in self.parts)
    
    def beg_includes(self, name: str) -> bool:
        name = name.strip('/')
        if len(self.parts):
            return name == self.parts[0] 
        else:
            return False
    
    @property
    def namespace(self):
        return self.build()
    
class HardwareNamespace(Namespace):
    def __init__(self, *parts):
        super().__init__(*parts)

    @property
    def is_simulation(self) -> bool:
        return self.beg_includes("sim")